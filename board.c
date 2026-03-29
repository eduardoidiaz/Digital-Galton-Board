
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

// Wall detection
#define hitBottom(b) (b>int2fix15(380))
#define hitTop(b) (b<int2fix15(100))
#define hitLeft(a) (a<int2fix15(100))
#define hitRight(a) (a>int2fix15(540))

// uS per frame
#define FRAME_RATE 33000

// Galton Board Parameters
#define GALTON_BOARD_START_X 320
#define GALTON_BOARD_START_Y 250
#define GALTON_BOARD_ROWS 10

#define PEG_RADIUS 3

#define NUM_BALLS 1
#define NUM_PEGS (GALTON_BOARD_ROWS*(GALTON_BOARD_ROWS + 1))/2

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

// the color of the boid
char color = WHITE ;

// Ball on core 0
struct Ball ball0;

struct Ball ball_array[NUM_BALLS];

struct Peg peg_array[NUM_PEGS];

volatile fix15 dx = 0;
volatile fix15 dy = 0;
volatile float distance = 0;
volatile fix15 normal_x = 0;
volatile fix15 normal_y = 0;
volatile fix15 intermediate = 0;
volatile struct Peg *last_peg;

double min = -2.0;
double max = 2.0;

// Create a ball dropping it from the center of screen
void spawn_ball(struct Ball *b) {
    b->x = int2fix15(320);
    b->y = int2fix15(248);
    b->vx = int2fix15((rand() % 5) + 1);
    b->vy = int2fix15(0);
    b->r = int2fix15(1);
}

// Draw the boundaries
void drawArena() {
    drawVLine(100, 100, 280, WHITE) ;
    drawVLine(540, 100, 280, WHITE) ;
    drawHLine(100, 100, 440, WHITE) ;
    drawHLine(100, 380, 440, WHITE) ;
}

// Detect bottoming out
void bottoming_out(struct Ball *b) {
    // If the ball bottoms out 'respawn'
    if (hitBottom(b->y)) {
        // Start in the center of screen
        b->x = int2fix15(320);
        b->y = int2fix15(240);
        b->vx = int2fix15((rand() % 2) + 1);
        b->vx = float2fix15(min + ((double)rand() / RAND_MAX) * (max - min));
        // Moving down
        b->vy = int2fix15(0);
    }
    // Update position using velocity
    b->x = b->x + b->vx;
    b->y = b->y + b->vy;
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

    // Spawn a ball
    // spawn_ball(&ball0);
    for (int i=0; i<NUM_BALLS; i++) {
        spawn_ball(&ball_array[i]);
    }

    while(1) {
        // Measure time at start of thread
        begin_time = time_us_32() ;      

        // erase ball
        // for (int i=0; i<NUM_BALLS; i++) {
        //     // drawRect(fix2int15(ball_array[i].x), fix2int15(ball_array[i].y), 2, 2, BLACK);
        //     drawCircle(fix2int15(ball_array[i].x), fix2int15(ball_array[i].y), fix2int15(ball_array[i].r), BLACK);
        //     fillCircle(fix2int15(ball_array[i].x), fix2int15(ball_array[i].y), fix2int15(ball_array[i].r), BLACK);
        // }

        for (int b=0; b<NUM_BALLS; b++) {
            drawCircle(fix2int15(ball_array[b].x), fix2int15(ball_array[b].y), fix2int15(ball_array[b].r), BLACK);
            fillCircle(fix2int15(ball_array[b].x), fix2int15(ball_array[b].y), fix2int15(ball_array[b].r), BLACK);
            for (int p=0; p<NUM_PEGS; p++) {
                dx = ball_array[b].x - peg_array[p].x;
                dy = ball_array[b].y - peg_array[p].y;
                // Are both the x and y distances less than the collision distance?
                if ((absfix15(dx) < (ball_array[b].r + peg_array[p].r)) && (absfix15(dy) < (ball_array[b].r + peg_array[p].r))) {
                    // If so, compute the distance separating the ball and peg
                    distance = sqrt(dx*dx + dy*dy);
                    
                    // Generate the normal vector that points from peg to ball
                    normal_x = divfix(dx, float2fix15(distance));
                    normal_y = divfix(dy, float2fix15(distance));

                    // Collision Physics 
                    intermediate = multfix15(int2fix15(-2), (multfix15(normal_x, ball_array[b].vx) + multfix15(normal_y, ball_array[b].vy)));

                    // Are ball velocity and normal vectors in opposite directions?
                    if (intermediate > 0) {
                        // Teleport it outside the collision distance with the peg
                        ball_array[b].x = peg_array[p].x + multfix15(normal_x, float2fix15(distance + 1));
                        ball_array[b].y = peg_array[p].y + multfix15(normal_y, float2fix15(distance + 1));

                        // Update its velocity
                        ball_array[b].vx = ball_array[b].vx + multfix15(normal_x, intermediate);
                        ball_array[b].vy = ball_array[b].vy + multfix15(normal_y, intermediate);

                        // Did we just strike a new peg?
                        if (&peg_array[p] != last_peg) {
                            // Make a sound

                            // Remove some energy from the ball
                            ball_array[b].vx = 1.3 * ball_array[b].vx;
                            ball_array[b].vy = 1.3 * ball_array[b].vy;
                        } 
                    }
                }
                last_peg = &peg_array[p];
            }
            // Respawn any balls that fall thru bottom
            bottoming_out(&ball_array[b]);

            // Bounce any balls the hit top/sides
            // TODO...
            if (fix2int15(ball_array[b].y) + 2*fix2int15(ball_array[b].x) < 870) {
                ball_array[b].vx = -ball_array[b].vx;
            }
            if (fix2int15(ball_array[b].y) - 2*fix2int15(ball_array[b].x) < -410) {
                ball_array[b].vx = -ball_array[b].vx;
            }

            // Apply gravity
            // ball_array[b].vy = ball_array[b].vy + int2fix15(1);
            ball_array[b].vy = ball_array[b].vy + float2fix15(0.15);

            // drawCircle(fix2int15(ball_array[b].x), fix2int15(ball_array[b].y), fix2int15(ball_array[b].r), BLACK);
            // fillCircle(fix2int15(ball_array[b].x), fix2int15(ball_array[b].y), fix2int15(ball_array[b].r), BLACK);

            // Use ball's updated velocity to update its position
            ball_array[b].x = ball_array[b].x + ball_array[b].vx;
            ball_array[b].y = ball_array[b].y + ball_array[b].vy;

            drawCircle(fix2int15(ball_array[b].x), fix2int15(ball_array[b].y), fix2int15(ball_array[b].r), color);
            fillCircle(fix2int15(ball_array[b].x), fix2int15(ball_array[b].y), fix2int15(ball_array[b].r), color);
        }
        // erase ball
        // for (int i=0; i<NUM_BALLS; i++) {
        //     // drawRect(fix2int15(ball_array[i].x), fix2int15(ball_array[i].y), 2, 2, BLACK);
        //     drawCircle(fix2int15(ball_array[i].x), fix2int15(ball_array[i].y), fix2int15(ball_array[i].r), BLACK);
        //     fillCircle(fix2int15(ball_array[i].x), fix2int15(ball_array[i].y), fix2int15(ball_array[i].r), BLACK);
        // }
        // detect ball bottoming out
        // bottoming_out(&ball0);
        // for (int i=0; i<NUM_BALLS; i++) {
        //     bottoming_out(&ball_array[i]);
        // }
        // draw the ball at its new position
        // for (int i=0; i<NUM_BALLS; i++) {
        //     // drawRect(fix2int15(ball_array[i].x), fix2int15(ball_array[i].y), 2, 2, color);
        //     drawCircle(fix2int15(ball_array[i].x), fix2int15(ball_array[i].y), fix2int15(ball_array[i].r), color);
        //     fillCircle(fix2int15(ball_array[i].x), fix2int15(ball_array[i].y), fix2int15(ball_array[i].r), color);
        // }
        // draw the boundaries
        drawArena() ;
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

    // Starting Center of Screen
    int x_coord = GALTON_BOARD_START_X;
    int y_coord = GALTON_BOARD_START_Y;
    int x_coord_inc = 14;
    int y_coord_inc = 14;
    int x_start_adj = x_coord_inc/2;
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
            drawCircle(x_coord, y_coord, PEG_RADIUS, WHITE);
            fillCircle(x_coord, y_coord, PEG_RADIUS, WHITE);
            // if (peg == 0) {
            //     drawPixel(x_coord - 10, y_coord, WHITE);
            // }
            // if (peg == row) {
            //     drawPixel(x_coord + 10, y_coord, WHITE);
            // }
            if (row == 0 && peg == 0) {
                top_peg_x = x_coord;
                top_peg_y = y_coord;
            }
            if (row == 9 && peg == 0) {
                bottom_left_peg_x = x_coord;
                bottom_left_peg_y = y_coord;
            }
            if (row == 9 && peg == row) {
                bottom_right_peg_x = x_coord;
                bottom_right_peg_y = y_coord;
            }
            peg_array[peg_idx].x = x_coord;
            peg_array[peg_idx].y = y_coord;
            peg_array[peg_idx].r = PEG_RADIUS;
            peg_idx += 1;
            if (row > 0) {
                x_coord += x_coord_inc; // increment x-coordinate to separate pegs horizontally
            }
        }
        y_coord += y_coord_inc; // move next drawing location vertically
    }
    last_peg = &peg_array[0];

    // sleep_ms(5000);
    // printf("top_peg(x, y): (%d, %d)\n", top_peg_x, top_peg_y);
    // printf("bottom_left_peg(x, y): (%d, %d)\n", bottom_left_peg_x, bottom_left_peg_y);
    // printf("bottom_right_peg(x, y): (%d, %d)\n", bottom_right_peg_x, bottom_right_peg_y);

    int top_x = GALTON_BOARD_START_X;
    int top_y = GALTON_BOARD_START_Y - 20;
    int bottom_left_x = bottom_left_peg_x - 10;
    int bottom_left_y = bottom_left_peg_y;
    int bottom_right_x = bottom_right_peg_x + 10;
    int bottom_right_y = bottom_right_peg_y;
    // m = (y2 - y1)/(x2 - x1)
    int slope_left = (top_y - bottom_left_y)/(top_x - bottom_left_x);
    int slope_right = (top_y - bottom_right_y)/(top_x - bottom_right_x);
    // y = mx + b -> b = y - mx
    int b_left = top_y - slope_left*top_x;
    int b_right = top_y - slope_right*top_x;


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

    // start core 1 
    multicore_reset_core1();
    multicore_launch_core1(&core1_main);

    // add threads
    pt_add_thread(protothread_serial);
    pt_add_thread(protothread_anim);

    // start scheduler
    pt_schedule_start ;
} 