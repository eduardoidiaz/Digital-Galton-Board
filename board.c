
/**
 * Eduardo Diaz
 * 
 * Inspired by: https://vanhunteradams.com/Pico/Galton/Galton.html
 * 
 * This program animates the collision physics of a Galton Board, demonstrating five
 * famous mathematical concepts: Bernoilli trials, the binomial distribution, the
 * Gaussian distribution, Pascal's triangle, and the central limit theorem.
 * Whenever any ball strikes a peg a 'thunk!' sound is made. This is done through
 * a DMA channel which avoids wasting cycles doing interrupt-based audio synthesis.
 * Featuring a potentiometer-based interface to allow for parameter tuning in realtime.
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
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
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

// uS per frame
#define FRAME_RATE 33000

// Galton Board Parameters
#define GALTON_BOARD_START_X 320
#define GALTON_BOARD_START_Y 0
#define GALTON_BOARD_ROWS 16
#define PEG_RADIUS 3
#define NUM_BALLS 10
#define NUM_PEGS (GALTON_BOARD_ROWS*(GALTON_BOARD_ROWS + 1))/2
# define NUM_BINS GALTON_BOARD_ROWS + 1

// Ball Parameters
#define BALL_RADIUS 1
#define BALL_START_X 320
#define BALL_START_Y 10
#define BALL_START_VY 0

#define GRAVITY 0.25

// Ball properties
struct Ball {
    fix15 x;
    fix15 y;
    fix15 vx;
    fix15 vy;
    fix15 r;
};

// Peg Properties
struct Peg {
    fix15 x;
    fix15 y;
    fix15 r;
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
volatile struct Peg *last_peg;

double min = -1.0;
double max = 1.0;

volatile int m_left = 0;
volatile int b_lft = 0;
volatile int m_right = 0;
volatile int b_rght = 0;

volatile int collisions = 0;

volatile int bottom = 0;

volatile int num = 0;
volatile int start = 0;
volatile int bin = 0;
volatile int balls_respawned = 0;

// Create a ball dropping it from the center of screen
void spawn_ball(struct Ball *b) {
    b->x = int2fix15(BALL_START_X);
    b->y = int2fix15(BALL_START_Y);
    b->vx = float2fix15(min + ((double)rand() / RAND_MAX) * (max - min));
    b->vy = int2fix15(BALL_START_VY);
    b->r = int2fix15(BALL_RADIUS);
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
        balls_respawned += 1;
    }
}

// Display info
void display() {

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
                        // if (ball->last_peg_id != peg->id) {
                        //     dma_trigger();
                        //     ball->last_peg_id = peg->id;

                        //     // Apply energy loss
                        //     ball->vx *= bounciness;
                        //     ball->vy *= bounciness;
                        // }
                        if (&peg_array[p] != last_peg) {
                            // Make a sound

                            // Remove some energy from the ball
                            ball_array[b].vx = 0.25 * ball_array[b].vx;
                            ball_array[b].vy = 0.25 * ball_array[b].vy;
                        } 
                        last_peg = &peg_array[p];
                    }
                }   
            }
            // Respawn any balls that fall thru bottom
            bottoming_out(&ball_array[b]);

            // Bounce any balls the hit top/sides
            // TODO...

            if (fix2int15(ball_array[b].y) - m_left*fix2int15(ball_array[b].x) < b_lft) {
                if (fix2int15(ball_array[b].y) < m_left*fix2int15(ball_array[b].x) + b_lft) {
                    ball_array[b].vy = -ball_array[b].vy;
                }
                // if (fix2int15(ball_array[b].y) < m_left*fix2int15(ball_array[b].x) + b_lft) {
                //     ball_array[b].vx = -ball_array[b].vx;
                // }
                ball_array[b].vx = -ball_array[b].vx;
            }
            if (fix2int15(ball_array[b].y) - m_right*fix2int15(ball_array[b].x) < b_rght) {
                if (fix2int15(ball_array[b].y) < m_right*fix2int15(ball_array[b].x) + b_rght) {
                    ball_array[b].vy = -ball_array[b].vy;
                }
                ball_array[b].vx = -ball_array[b].vx;
            }

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
        // Measure time at start of thread
        begin_time = time_us_32() ;      

        // Redraw PEGS
        for (int i=0; i<NUM_PEGS; i++) {
            drawCircle(fix2int15(peg_array[i].x), fix2int15(peg_array[i].y) , PEG_RADIUS, WHITE);
        }

        // Draw bins filling
        for (int i=0; i<NUM_BINS; i++) {
            if (bins_array[i].num_balls > 0) {
                // fillRect(fix2int15(bins_array[i].x_i)+(7), 470 - bins_array[i].num_balls, (fix2int15(bins_array[0].x_f) - fix2int15(bins_array[0].x_i))-1, bins_array[i].num_balls, GREEN);
                fillRect(fix2int15(bins_array[i].x_i)+1, 470 - bins_array[i].num_balls, (fix2int15(bins_array[0].x_f) - fix2int15(bins_array[0].x_i))-1, bins_array[i].num_balls, GREEN);
            }
        }

        // Write Hello
        char *str = "HELLO!";
        int str_len = 6;
        char *str1 = "BIN #";
        char str2[str_len];

        // Clear Text
        drawRect(0, 0, 100, 150, BLACK);
        fillRect(0, 0, 100, 150, BLACK);
        setCursor(0, 10);
        setTextColor(WHITE);
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

    // initialize VGA
    initVGA() ;

    // white screen
    drawRect(0, 0, 640, 480, WHITE);
    fillRect(0, 0, 640, 480, WHITE);

    // Starting Center of Screen
    int x_coord = GALTON_BOARD_START_X;
    int y_coord = GALTON_BOARD_START_Y;
    int x_coord_inc = 14;
    int y_coord_inc = 14;
    int x_start_adj = x_coord_inc/2;
    // int x_start_adj = 14;
    int peg_idx = 0;
    int bottom_left_peg_x = 0;
    int bottom_left_peg_y = 0;
    int bottom_right_peg_x = 0;
    int bottom_right_peg_y = 0;
    int top_peg_x = 0;
    int top_peg_y = 0;

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
            peg_idx += 1;
            if (row > 0) {
                x_coord += x_coord_inc; // increment x-coordinate to separate pegs horizontally
            }
        }
        y_coord += y_coord_inc; // move next drawing location vertically
    }

    int top_x = GALTON_BOARD_START_X;
    int top_y = GALTON_BOARD_START_Y - 2*y_coord_inc;
    // int bottom_left_x = bottom_left_peg_x - 10;
    int bottom_left_x = bottom_left_peg_x - x_coord_inc;
    int bottom_left_y = bottom_left_peg_y;
    // int bottom_left_y = bottom_left_peg_y + y_coord_inc;
    // int bottom_right_x = bottom_right_peg_x + 10;
    int bottom_right_x = bottom_right_peg_x + x_coord_inc;
    int bottom_right_y = bottom_right_peg_y;
    // m = (y2 - y1)/(x2 - x1)
    int slope_left = (top_y - bottom_left_y)/(top_x - bottom_left_x);
    m_left = slope_left;
    int slope_right = (top_y - bottom_right_y)/(top_x - bottom_right_x);
    m_right = slope_right;
    // y = mx + b -> b = y - mx
    int b_left = top_y - slope_left*top_x;
    b_lft = b_left;
    int b_right = top_y - slope_right*top_x;
    b_rght = b_right;

    bottom = bottom_left_peg_y;

    // Draw Left Diagonal of Galton Board (starting from bottom left)
    for (int y = bottom_left_y; y >= top_y; y--) {
        for (int x = bottom_left_x; x <= top_x; x++) {
            if (y-slope_left*x == b_left) {
                drawPixel(x, y, WHITE);
            } else if (y-slope_left*x < b_left) {
                drawPixel(x, y, RED);
            } else {
                // drawPixel(x, y, GREEN);
            }
        }
    }
    // Draw Right Diagonal of Galton Board (starting from bottom right)
    for (int y = bottom_right_peg_y; y >= top_y; y--) {
        for (int x = bottom_right_x; x >= top_x; x--) {
            if (y-slope_right*x == b_right) {
                drawPixel(x, y, WHITE);
            } else if (y-slope_right*x < b_right) {
                drawPixel(x, y, RED);
            } else {
                // drawPixel(x, y, GREEN);
            }
        }
    }

    for (int i=0; i<NUM_BALLS; i++) {
        spawn_ball(&ball_array[i]);
    }

    num = (GALTON_BOARD_ROWS*(GALTON_BOARD_ROWS+1)/2) - 1;
    start = (GALTON_BOARD_ROWS*(GALTON_BOARD_ROWS+1)/2 - GALTON_BOARD_ROWS);

    // bins_array[bin].x_i = peg_array[start].x - int2fix15(10);
    // bins_array[bin].x_f = peg_array[start].x;
    // bins_array[bin+10].x_i = peg_array[54].x;
    // bins_array[bin+10].x_f = peg_array[54].x + int2fix15(10);

    for (int i=0; i < GALTON_BOARD_ROWS+1; i++) {
        if (i==0) { // first peg in last row
            // bins_array[i].x_i = peg_array[start+i].x - int2fix15(10);
            bins_array[i].x_i = peg_array[start+i].x - int2fix15(x_coord_inc);
            bins_array[i].x_f = peg_array[start+i].x;   
        } else if (i==GALTON_BOARD_ROWS) { // last peg in last row
            bins_array[i].x_i = peg_array[start+i-1].x;
            // bins_array[i].x_f = peg_array[start+i-1].x  + int2fix15(10);
            bins_array[i].x_f = peg_array[start+i-1].x  + int2fix15(x_coord_inc);
        } else {
            bins_array[i].x_i = peg_array[start+i-1].x;
            bins_array[i].x_f = peg_array[start+i].x;
        }
        bins_array[i].num_balls = 0;
    }
    

    sleep_ms(3000);

    // start core 1 
    multicore_reset_core1();
    multicore_launch_core1(&core1_main);

    // add threads
    pt_add_thread(protothread_serial);
    pt_add_thread(protothread_anim);

    // start scheduler
    pt_schedule_start ;
} 