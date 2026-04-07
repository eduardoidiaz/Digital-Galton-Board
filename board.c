
/**
 * Eduardo Diaz
 * 
 * This demonstration animates two balls bouncing about the screen.
 * Through a serial interface, the user can change the ball color.
 *
 * HARDWARE CONNECTIONS
  - GPIO 16 ---> VGA Hsync
  - GPIO 17 ---> VGA Vsync
  - GPIO 18 ---> VGA Green lo-bit --> 470 ohm resistor --> VGA_Green
  - GPIO 19 ---> VGA Green hi_bit --> 330 ohm resistor --> VGA_Green
  - GPIO 20 ---> 330 ohm resistor ---> VGA-Blue
  - GPIO 21 ---> 330 ohm resistor ---> VGA-Red
  - RP2040 GND ---> VGA-GND
 *
 * RESOURCES USED
 *  - PIO state machines 0, 1, and 2 on PIO instance 0
 *  - DMA channels (2, by claim mechanism)
 *  - 153.6 kBytes of RAM (for pixel color data)
 *
 */

// Include the VGA grahics library
#include "vga16_graphics_v2.h"
// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
// Include Pico libraries
#include "pico/stdlib.h"
#include "pico/divider.h"
#include "pico/multicore.h"
// Include hardware libraries
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/spi.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "hardware/adc.h"
// Include protothreads
#include "pt_cornell_rp2040_v1_4.h"

// === the fixed point macros ========================================
typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define float2fix15(a) ((fix15)((a)*32768.0)) // 2^15
#define fix2float15(a) ((float)(a)/32768.0)
#define absfix15(a) abs(a) 
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a,b) (fix15)(div_s64s64( (((signed long long)(a)) << 15), ((signed long long)(b))))

// Wall detection
#define hitBottom(b) (b>int2fix15(380))
#define hitTop(b) (b<int2fix15(100))
#define hitLeft(a) (a<int2fix15(100))
#define hitRight(a) (a>int2fix15(540))

// uS per frame
#define FRAME_RATE 33000

// Galton Board Parameters
#define GALTON_BOARD_START_X 320
#define GALTON_BOARD_START_Y 30
#define GALTON_BOARD_ROWS 16
#define PEG_SEPARATION_X 30
#define PEG_SEPARATION_Y 15
#define PEG_RADIUS 3
#define NUM_BALLS 100
#define NUM_PEGS (GALTON_BOARD_ROWS*(GALTON_BOARD_ROWS + 1))/2
#define NUM_BINS GALTON_BOARD_ROWS + 1
#define DIAGONAL_START_X GALTON_BOARD_START_X
#define DIAGONAL_START_Y GALTON_BOARD_START_Y - PEG_SEPARATION_Y
#define WALL_BOUNCE float2fix15(0.6)   // tweak: 0.4–0.8 feels good
// #define GRAVITY 0.125
#define GRAVITY 0.75
#define BOUNCINESS 0.5

// Ball Parameters
#define BALL_RADIUS 2
#define BALL_START_X GALTON_BOARD_START_X 
#define BALL_START_Y GALTON_BOARD_START_Y - 15
#define BALL_START_VY 0

#define BIN_START_Y 470

// Number of DMA transfers per event
#define THUNK_SIZE 256

volatile int animated_balls = 0;

// Table of values to be sent to DAC
unsigned short DAC_data[THUNK_SIZE] ;

// Pointer to the address of the DAC data table
unsigned short * address_ptr = &DAC_data[0] ;

// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000

//SPI configurations
#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SCK  6
#define PIN_MOSI 7
#define SPI_PORT spi0

volatile int b_left = 0;
volatile int left_diag_slope = 0;
volatile int b_right = 0;
volatile int right_diag_slope = 0;

const uint adc_max = (1<<12) - 1; // 4095
volatile int adc_avg = 0;
volatile uint16_t potent_val = 0;
volatile float potent_pecentage = 0.0;
volatile bool read_adc = false;


// Ball properties
struct Ball {
    fix15 x;
    fix15 y;
    fix15 vx;
    fix15 vy;
    fix15 r;
    int last_peg_id;
};

// Peg Properties
struct Peg {
    fix15 x;
    fix15 y;
    fix15 r;
    int id;
};

// Bin Properties
struct Bin {
    // Initial/Final x-coordinates for bin
    fix15 x_i, x_f;
    int num_balls;
};

// the color of the boid
char color = WHITE ;

// Ball on core 0
struct Ball ball0;

struct Ball ball_array[NUM_BALLS];

struct Peg peg_array[NUM_PEGS];

struct Bin bins_array[NUM_BINS];

volatile fix15 dx = 0;
volatile fix15 dy = 0;
volatile float distance = 0;
volatile fix15 normal_x = 0;
volatile fix15 normal_y = 0;
volatile fix15 intermediate = 0;

double min = -1.0;
double max = 1.0;

volatile int collisions = 0;

volatile int bottom = 0;

// Select DMA channels
volatile int data_chan = 0;

// Create a ball dropping it from the center of screen
void spawn_ball(struct Ball *b) {
    b->x = int2fix15(BALL_START_X);
    b->y = int2fix15(BALL_START_Y);
    b->vx = float2fix15(min + ((double)rand() / RAND_MAX) * (max - min));
    b->vy = int2fix15(BALL_START_VY);
    b->r = int2fix15(BALL_RADIUS);
    b->last_peg_id = -1; // invalid peg
}

// ==================================================
// Wall collision (fixed-point, no floats)
// ==================================================
static inline void wall_collision(struct Ball *b) {

    // Convert once (faster)
    int x = fix2int15(b->x);
    int y = fix2int15(b->y);

    fix15 vx = b->vx;
    fix15 vy = b->vy;

    // ---------- LEFT WALL ----------
    // d = 2x + y - 646
    int d_left = (left_diag_slope * x - y + b_left);

    if (d_left > 0) {

        // Normal = (2,1)
        fix15 dot = multfix15(vx, int2fix15(2)) + multfix15(vy, int2fix15(1));

        // Only reflect if moving into wall
        if (dot < int2fix15(0)) {
            // |n|^2 = 2^2 + 1^2 = 5

            // Reflect velocity: v = v - 2*(dot/5)*n
            fix15 scale = divfix(multfix15(int2fix15(2), dot), int2fix15(5));

            vx -= multfix15(scale, int2fix15(2));
            vy -= multfix15(scale, int2fix15(1));

            // Write back
            // APPLY ENERGY LOSS
            b->vx = multfix15(WALL_BOUNCE, vx);
            b->vy = multfix15(WALL_BOUNCE, vy);
        }

        // Push away from wall (prevents sticking)
        int push = (d_left + 1);  // +1 = bias outward

        // Move along normal
        x += (push * 2) / 5;
        y += (push * 1) / 5;

        b->x = int2fix15(x);
        b->y = int2fix15(y);
    }

    // ---------- RIGHT WALL ----------
    // d = -2x + y + 634
    int d_right = (right_diag_slope * x - y + b_right);

    if (d_right > 0) {

        // Normal = (-2,1)
        fix15 dot = multfix15(vx, int2fix15(-2)) + multfix15(vy, int2fix15(1));

        if (dot < int2fix15(0)) {
            // |n|^2 = 5
            fix15 scale = divfix(multfix15(int2fix15(2), dot), int2fix15(5));

            vx -= multfix15(scale, int2fix15(-2));
            vy -= multfix15(scale, int2fix15(1));

            // APPLY ENERGY LOSS
            b->vx = multfix15(WALL_BOUNCE, vx);
            b->vy = multfix15(WALL_BOUNCE, vy);
        }

        // Push away from wall (prevents sticking)
        int push = (d_right + 1);

        x += (push * -2) / 5;
        y += (push *  1) / 5;

        b->x = int2fix15(x);
        b->y = int2fix15(y);
    }
}
// ==================================================
// Draw diagonal Galton walls
// ==================================================
void draw_diagonal_walls() {

    int y_top = DIAGONAL_START_Y; // 6
    int y_bot = bottom;                                // ~160

    // for (int y = y_top; y <= y_bot; y++) {

    //     // From equations:
    //     // Left:  y = -2x + 646  → x = (646 - y)/2
    //     int x_left  = (-y + b_left) >> 1;

    //     // Right: y =  2x - 634  → x = (y + 634)/2
    //     int x_right = (y - b_right) >> 1;

    //     // Draw pixels
    //     drawPixel(x_left,  y, WHITE);
    //     drawPixel(x_right, y, WHITE);
    // }

    // Draw Left Diagonal of Galton Board (starting from bottom left)
    for (int y = y_bot; y >= y_top; y--) {
        for (int x = fix2int15(bins_array[0].x_i); x <= GALTON_BOARD_START_X; x++) {
            if (y-(left_diag_slope*x) == b_left) {
                drawPixel(x, y, WHITE);
            } else if (y-(left_diag_slope*x) < b_left) {
                // drawPixel(x, y, RED);
            } else {
                // drawPixel(x, y, GREEN);
            }
        }
    }
    
    // Draw Right Diagonal of Galton Board (starting from bottom right)
    for (int y = y_bot; y >= y_top; y--) {
        for (int x = fix2int15(bins_array[NUM_BINS-1].x_f); x >= GALTON_BOARD_START_X; x--) {
            if (y-(right_diag_slope*x) == b_right) {
                drawPixel(x, y, WHITE);
            } else if (y-(right_diag_slope*x) < b_right) {
                // drawPixel(x, y, RED);
            } else {
                // drawPixel(x, y, GREEN);
            }
        }
    }
}


// Detect bottoming out
void bottoming_out(struct Ball *b) {
    // If the ball bottoms out 'respawn'
    if (b->y > int2fix15(bottom)) {
        // Check which bin ball fell into
        for (int i=0; i<NUM_BINS; i++) {
            if ((b->x < bins_array[i].x_f) && (b->x > bins_array[i].x_i)) {
                bins_array[i].num_balls = bins_array[i].num_balls + 1;
            }
        }
        // Start in the center of screen
        b->x = int2fix15(BALL_START_X);
        b->y = int2fix15(BALL_START_Y);
        b->vx = float2fix15(min + ((double)rand() / RAND_MAX) * (max - min));
        // Moving down
        b->vy = int2fix15(BALL_START_VY);
    }
}

// ==================================================
// === users serial input thread
// ==================================================
static PT_THREAD (protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt);
    // stores user input
    static int user_input ;
    // wait for 0.1 sec
    PT_YIELD_usec(1000000) ;
    // announce the threader version
    sprintf(pt_serial_out_buffer, "Protothreads RP2040 v1.0\n\r");
    // non-blocking write
    serial_write ;
    while(1) {
        // print prompt
        sprintf(pt_serial_out_buffer, "input a number in the range 1-15: ");
        // non-blocking write
        serial_write ;
        // spawn a thread to do the non-blocking serial read
        serial_read ;
        // convert input string to number
        sscanf(pt_serial_in_buffer,"%d", &user_input) ;
        // update boid color
        if ((user_input > 0) && (user_input < 16)) {
            color = (char)user_input ;
        }
    } // END WHILE(1)
    PT_END(pt);
} // timer thread

// GPIO ISR. Toggles LED
void gpio_callback() {
    gpio_put(25, !gpio_get(25));

    // Read ADC
    read_adc = true;
    // adc_select_input(2);
}


// Animation on core 0
static PT_THREAD (protothread_anim(struct pt *pt))
{
    // Mark beginning of thread
    PT_BEGIN(pt);

    // Variables for maintaining frame rate
    static int begin_time ;
    static int spare_time ;

    while(1) {
        // Measure time at start of thread
        begin_time = time_us_32() ;      

        for (int b=0; b<NUM_BALLS; b++) {
            if (b>=animated_balls) {
                // Erase Ball (inactive)
                drawCircle(fix2int15(ball_array[b].x), fix2int15(ball_array[b].y), fix2int15(ball_array[b].r), BLACK);
                fillCircle(fix2int15(ball_array[b].x), fix2int15(ball_array[b].y), fix2int15(ball_array[b].r), BLACK);
                spawn_ball(&ball_array[b]);
                continue;
            }
            // Erase Ball
            drawCircle(fix2int15(ball_array[b].x), fix2int15(ball_array[b].y), fix2int15(ball_array[b].r), BLACK);
            fillCircle(fix2int15(ball_array[b].x), fix2int15(ball_array[b].y), fix2int15(ball_array[b].r), BLACK);
            for (int p=0; p<NUM_PEGS; p++) {

                float dx = fix2float15(ball_array[b].x) - fix2float15(peg_array[p].x);
                float dy = fix2float15(ball_array[b].y) - fix2float15(peg_array[p].y);

                float radius_sum = fix2float15(ball_array[b].r) + fix2float15(peg_array[p].r);

                // Fast AABB-style early reject
                if (fabsf(dx) > radius_sum || fabsf(dy) > radius_sum) {
                    continue;
                }

                float dist_sq = dx * dx + dy * dy;
                float radius_sq = radius_sum * radius_sum;

                // Check actual collision
                if (dist_sq < radius_sq) {
                    // Avoid divide-by-zero
                    if (dist_sq < 1e-6f) {
                        continue;
                    }

                    // Dot product (velocity toward peg?)
                    float dot = dx * fix2float15(ball_array[b].vx) + dy * fix2float15(ball_array[b].vy);

                    // Only reflect if moving toward peg
                    if (dot < 0.0f) {

                        // Reflection factor (no sqrt needed)
                        float factor = -2.0f * dot / dist_sq;

                        // Reflect velocity
                        ball_array[b].vx += float2fix15(dx * factor);
                        ball_array[b].vy += float2fix15(dy * factor);

                        // Push ball out of peg (minimal correction)
                        float dist = sqrtf(dist_sq);
                        float overlap = radius_sum - dist;

                        float nx = dx / dist;
                        float ny = dy / dist;

                        ball_array[b].x += float2fix15(nx * overlap);
                        ball_array[b].y += float2fix15(ny * overlap);

                        // Trigger sound once per new peg
                        if (ball_array[b].last_peg_id != peg_array[p].id) {
                            // Make a sound
                            dma_channel_set_read_addr(data_chan, DAC_data, true);

                            // Remove some energy from the ball
                            ball_array[b].vx = multfix15(float2fix15(BOUNCINESS), ball_array[b].vx);
                            ball_array[b].vy = multfix15(float2fix15(BOUNCINESS), ball_array[b].vy);
                        }
                        ball_array[b].last_peg_id = peg_array[p].id;
                    }
                }   
            }
            // Respawn any balls that fall thru bottom
            bottoming_out(&ball_array[b]);

            // Bounce any balls the hit top/sides
            // TODO...
            wall_collision(&ball_array[b]);

            // Apply gravity
            ball_array[b].vy = ball_array[b].vy + float2fix15(GRAVITY);

            // Use ball's updated velocity to update its position
            ball_array[b].x = ball_array[b].x + ball_array[b].vx;
            ball_array[b].y = ball_array[b].y + ball_array[b].vy;

            drawCircle(fix2int15(ball_array[b].x), fix2int15(ball_array[b].y), fix2int15(ball_array[b].r), color);
            fillCircle(fix2int15(ball_array[b].x), fix2int15(ball_array[b].y), fix2int15(ball_array[b].r), color);
        }
        // delay in accordance with frame rate
        spare_time = FRAME_RATE - (time_us_32() - begin_time) ;
        // yield for necessary amount of time
        PT_YIELD_usec(spare_time) ;
        // NEVER exit while
    } // END WHILE(1)
    PT_END(pt);
} // animation thread


// Animation on core 1
static PT_THREAD (protothread_anim1(struct pt *pt))
{
    // Mark beginning of thread
    PT_BEGIN(pt);

    // Variables for maintaining frame rate
    static int begin_time ;
    static int spare_time ;

    while(1) {
        if (read_adc) {
            adc_select_input(2);
            for (int i=0; i<256; i++) {
                adc_avg += adc_read();
            }
            potent_val = adc_avg / 256;
            potent_pecentage = ((float) potent_val / (float) adc_max) * 100.0;
            printf("potent_percentage = %d\n", (int) potent_pecentage);
            animated_balls = (int) potent_pecentage;
            adc_avg = 0;
        }
        // printf("peg_array[0].x = %d\n", fix2int15(peg_array[0].x));
        // printf("peg_array[0].y = %d\n", fix2int15(peg_array[0].y)-PEG_SEPARATION);
        // printf("bin[0]_xi = %d\n", fix2int15(bins_array[0].x_i));
        // printf("bin[NUM_BINS-1]_xf = %d\n", fix2int15(bins_array[NUM_BINS-1].x_f));
        // printf("bottom = %d\n\n", bottom);
        // printf("left_diag_slope = %d\n", left_diag_slope);
        // printf("b_left = %d\n", b_left);

        // Measure time at start of thread
        begin_time = time_us_32() ;      

        // Redraw PEGS
        for (int i=0; i<NUM_PEGS; i++) {
            drawCircle(fix2int15(peg_array[i].x), fix2int15(peg_array[i].y) , PEG_RADIUS, WHITE);
        }

        // Draw walls enclosing the Galton Board
        draw_diagonal_walls();

        // Draw bins;
        for (int i=0; i<NUM_BINS; i++) {
            drawVLine(fix2int15(bins_array[i].x_i), bottom, 10, WHITE);
            drawVLine(fix2int15(bins_array[i].x_f), bottom, 10, WHITE);

            // Draw bins filling
            if (bins_array[i].num_balls > 0) {
                fillRect(fix2int15(bins_array[i].x_i)+1, BIN_START_Y - bins_array[i].num_balls, fix2int15(bins_array[i].x_f - bins_array[i].x_i) - 1, bins_array[i].num_balls, GREEN);
            }
        }

        char *str1 = "BIN #";
        int str_len = 6;
        char str2[str_len];

        // // Clear Text
        drawRect(0, 0, 120, 200, WHITE);
        fillRect(0, 0, 120, 200, WHITE);
        setCursor(0, 5);
        setTextColor(BLACK);
        setTextSize(1);

        for (int b=0; b<NUM_BINS; b++) {
            str2[0] = b + '0';
            str2[1] = ' ';
            str2[2] = bins_array[b].num_balls/10 % 10 + '0';
            str2[3] = bins_array[b].num_balls % 10 + '0';
            str2[4] = '\n';
            str2[5] = '\0';
            writeString(str1);
            writeString(str2);
        }

        char *str3 = "Animated Balls = ";
        char str4[4];
        for (int i=0; i<4; i++) {
            str4[0] = animated_balls/10 % 10 + '0';
            str4[1] = animated_balls % 10 + '0';
            str4[2] = '\n';
            str4[3] = '\0';
        }
        writeString(str3);
        writeString(str4);

        // delay in accordance with frame rate
        spare_time = FRAME_RATE - (time_us_32() - begin_time) ;
        // yield for necessary amount of time
        PT_YIELD_usec(spare_time) ;
        // NEVER exit while
    } // END WHILE(1)
    PT_END(pt);
} // animation thread

// ========================================
// === core 1 main -- started in main below
// ========================================
void core1_main(){
    // Add animation thread
    pt_add_thread(protothread_anim1);
    // Start the scheduler
    pt_schedule_start ;

}

// ========================================
// === main
// ========================================
// USE ONLY C-sdk library
int main(){
    set_sys_clock_khz(150000, true) ;
    // initialize stio
    stdio_init_all() ;

    // Configure GPIO interrupt
    gpio_set_irq_enabled_with_callback(2, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);

    // Set GPIO's 3 to output
    gpio_init(3) ;
    gpio_init(25) ;
    gpio_set_dir(2, GPIO_IN);
    gpio_set_dir(25, GPIO_OUT);

    // Set LED to off
    gpio_put(25, 0);

    // ADC init
    adc_init();
    adc_gpio_init(28);


    // initialize VGA
    initVGA() ;

    // Initialize SPI channel (channel, baud rate set to 20MHz)
    spi_init(SPI_PORT, 20000000) ;

    // Format SPI channel (channel, data bits per transfer, polarity, phase, order)
    spi_set_format(SPI_PORT, 16, 0, 0, 0);

    // Map SPI signals to GPIO ports, acts like framed SPI with this CS mapping
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI) ;
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    // Build DAC data table
    for (int i = 0; i < THUNK_SIZE; i++) {
        // Exponential decay envelope
        float decay = expf(-5.0f * i / THUNK_SIZE);

        // Low-frequency + noise mix (gives it that "impact" feel)
        // float tone = sinf(2 * M_PI * i / 20.0f);   // low freq bump
        float tone = sinf(2 * M_PI * i / 95.0f); // lower tone (deeper thunk)
        float noise = ((rand() % 2000) - 1000) / 1000.0f;

        float sample = (0.9f * tone + 0.1f * noise) * decay;
        // float sample = (0.4f * tone + 0.6f * noise) * decay; // enhance click

        // if (i < 8) sample *= 2.0f;  // punch at start

        // Scale to 12-bit DAC
        int value = (int)(2047 + 2047 * sample);

        if (value < 0) value = 0;
        if (value > 4095) value = 4095;

        DAC_data[i] = DAC_config_chan_A | (value & 0x0fff);
    }

    // Select DMA channels
    data_chan = dma_claim_unused_channel(true);

    // Setup the data channel
    dma_channel_config c2 = dma_channel_get_default_config(data_chan);  // Default configs
    channel_config_set_transfer_data_size(&c2, DMA_SIZE_16);            // 16-bit txfers
    channel_config_set_read_increment(&c2, true);                       // yes read incrementing
    channel_config_set_write_increment(&c2, false);                     // no write incrementing
    // (X/Y)*sys_clk, where X is the first 16 bytes and Y is the second
    // sys_clk is 125 MHz unless changed in code. Configured to ~44 kHz
    dma_timer_set_fraction(0, 0x0017, 0xffff) ;
    // 0x3b means timer0 (see SDK manual)
    channel_config_set_dreq(&c2, 0x3b);                                 // DREQ paced by timer 0

    dma_channel_configure(
        data_chan,                  // Channel to be configured
        &c2,                        // The configuration we just created
        &spi_get_hw(SPI_PORT)->dr,  // write address (SPI data register)
        DAC_data,                   // The initial read address
        THUNK_SIZE,                 // Number of transfers
        false                       // Don't start immediately.
    );

    // Starting Center of Screen
    int x_coord = GALTON_BOARD_START_X;
    int y_coord = GALTON_BOARD_START_Y;
    int x_coord_inc = PEG_SEPARATION_X;
    int y_coord_inc = PEG_SEPARATION_Y;
    int x_start_adj = x_coord_inc/2;
    int peg_idx = 0;
    int bottom_left_peg_x = 0;
    int bottom_left_peg_y = 0;
    int bottom_right_peg_x = 0;
    int bottom_right_peg_y = 0;
    int top_peg_x = 0;
    int top_peg_y = 0;
    int bin_idx = 1;

    for (int row=0; row<GALTON_BOARD_ROWS; row++) { // Rows in Galton Board
        x_coord = 320 - x_start_adj*row; // Move starting location of pegs (no movement initially)
        for (int peg=0; peg<row+1; peg++) { // Pegs per row in Galton Board (1 more than row number)
            if (row == 0 && peg == 0) {
                top_peg_x = x_coord;
                top_peg_y = y_coord;
            }
            if (row == GALTON_BOARD_ROWS-1 && peg == 0) {
                bottom_left_peg_x = x_coord;
                bottom_left_peg_y = y_coord;
            }
            if (row == GALTON_BOARD_ROWS-1 && peg == row) {
                bottom_right_peg_x = x_coord;
                bottom_right_peg_y = y_coord;
            }
            peg_array[peg_idx].x = int2fix15(x_coord);
            peg_array[peg_idx].y = int2fix15(y_coord);
            peg_array[peg_idx].r = int2fix15(PEG_RADIUS);
            peg_array[peg_idx].id = peg_idx;
            // Last row
            if (row==GALTON_BOARD_ROWS-1) {
                bins_array[bin_idx].x_i = peg_array[peg_idx].x;
                bins_array[bin_idx].x_f = peg_array[peg_idx].x + int2fix15(x_coord_inc);
                bins_array[bin_idx].num_balls = 0;
                bin_idx += 1;
            }
            peg_idx += 1;
            if (row > 0) {
                x_coord += x_coord_inc; // increment x-coordinate to separate pegs horizontally
            }
        }
        y_coord += y_coord_inc; // move next drawing location vertically
    }
    // Set initial bin coordinates
    bins_array[0].x_i = bins_array[1].x_i - int2fix15(x_coord_inc);
    bins_array[0].x_f = bins_array[1].x_i;
    bins_array[0].num_balls = 0;

    // Set the y-coordinate for the bottom of the Galton Board
    bottom = y_coord;

    left_diag_slope = (bottom - (DIAGONAL_START_Y)) / (fix2int15(bins_array[0].x_i) - DIAGONAL_START_X);
    b_left = DIAGONAL_START_Y - left_diag_slope*DIAGONAL_START_X;
    right_diag_slope = (bottom - (DIAGONAL_START_Y)) / (fix2int15(bins_array[NUM_BINS-1].x_f) - DIAGONAL_START_X);
    b_right = DIAGONAL_START_Y - (right_diag_slope*DIAGONAL_START_X);
    
    // Draw horizontal line where the galton board
    drawHLine(0, bottom, 640, WHITE);

    // Initialize balls to start from top of Galton Board
    for (int i=0; i<NUM_BALLS; i++) {
        spawn_ball(&ball_array[i]);
    }

    sleep_ms(1000);

    // start core 1 
    multicore_reset_core1();
    multicore_launch_core1(&core1_main);

    // add threads
    pt_add_thread(protothread_serial);
    pt_add_thread(protothread_anim);

    // start scheduler
    pt_schedule_start ;
} 