
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

// the color of the boid
char color = WHITE ;

// Ball on core 0
fix15 ball0_x ;
fix15 ball0_y ;
fix15 ball0_vx ;
fix15 ball0_vy ;

// Create a ball dropping it from the center of screen
void spawn_ball(fix15* x, fix15* y, fix15* vx, fix15* vy, int direction)
{
  // Start in center of screen
  *x = int2fix15(320) ;
  *y = int2fix15(240) ;
  // Choose left or right
//   if (direction) *vx = int2fix15(3) ;
//   else *vx = int2fix15(-3) ;
  // Moving down
  *vy = int2fix15(1) ;
}

// Draw the boundaries
void drawArena() {
  drawVLine(100, 100, 280, WHITE) ;
  drawVLine(540, 100, 280, WHITE) ;
  drawHLine(100, 100, 440, WHITE) ;
  drawHLine(100, 380, 440, WHITE) ;
}

// Detect bottoming out
void bottoming_out(fix15* x, fix15* y, fix15* vx, fix15* vy) {
    // If the ball bottoms out 'respawn'
    if (hitBottom(*y)) {
        // Start in center of screen
        *x = int2fix15(320) ;
        *y = int2fix15(240) ;
        // Moving down
        *vy = int2fix15(1) ;
    }
    // Update position using velocity
    *x = *x + *vx ;
    *y = *y + *vy ;
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
    spawn_ball(&ball0_x, &ball0_y, &ball0_vx, &ball0_vy, 0);

    while(1) {
      // Measure time at start of thread
      begin_time = time_us_32() ;      
      // erase ball
      drawRect(fix2int15(ball0_x), fix2int15(ball0_y), 2, 2, BLACK);
      // detect ball bottoming out
      bottoming_out(&ball0_x, &ball0_y, &ball0_vx, &ball0_vy);
      // draw the ball at its new position
      drawRect(fix2int15(ball0_x), fix2int15(ball0_y), 2, 2, color); 
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
//   int NUM_ROWS = GALTON_BOARD_ROWS;
  int x_coord = GALTON_BOARD_START_X;
  int y_coord = GALTON_BOARD_START_Y;
  int x_coord_inc = 14;
  int y_coord_inc = 14;
  int x_start_adj = x_coord_inc/2;

  for (int row=0; row<GALTON_BOARD_ROWS; row++) { // Rows in Galton Board
    x_coord = 320 - x_start_adj*row; // Move starting location of pegs (no movement initially)
    for (int peg=0; peg<row+1; peg++) { // Pegs per row in Galton Board (1 more than row number)
        drawCircle(x_coord, y_coord, 3, WHITE);
        fillCircle(x_coord, y_coord, 3, WHITE);
        if (row > 0) {
            x_coord += x_coord_inc; // increment x-coordinate to separate pegs horizontally
        }
    }
    y_coord += y_coord_inc; // move next drawing location vertically
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