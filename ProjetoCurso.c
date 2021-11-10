#include <stdio.h>
#include <stdlib.h>

#include <webots/robot.h>

#include <webots/motor.h>
#include <webots/distance_sensor.h>

#include <webots/supervisor.h>

/*
 * You may want to add macros here.
 */
#define TIME_STEP 128
#define QtddSensorDist 8

static WbDeviceTag motorE, motorD;

static WbDeviceTag groundE, groundD;
static double groundE_Valor, groundD_Valor;

#define LEFT 0
#define RIGHT 1
#define MAX_SPEED 3.14

static double offset = 0.5 * MAX_SPEED;

 
static int get_time_step() {
  static int time_step = -1;
  if (time_step == -1)
    time_step = (int)wb_robot_get_basic_time_step();
  return time_step;
}


void robot_wait(double seconds) {
  const double t = wb_robot_get_time();
  while (wb_robot_get_time() - t < seconds) {
    if (wb_robot_step(TIME_STEP) == -1) break;
  }
} 
 

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  double Velocidades[2];
  bool speed_set = false;
  
  /* necessary to initialize webots stuff */
  wb_robot_init();

  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */
  //MOTORES
  motorE = wb_robot_get_device("left wheel motor");
  motorD = wb_robot_get_device("right wheel motor");
  
  wb_motor_set_position(motorE, INFINITY);
  wb_motor_set_position(motorD, INFINITY);
  
  wb_motor_set_velocity(motorE, 0);
  wb_motor_set_velocity(motorD, 0);
  
  //sensor de solo  
  groundE = wb_robot_get_device("IR0");
  groundD = wb_robot_get_device("IR1");
  
  wb_distance_sensor_enable(groundE, get_time_step());
  wb_distance_sensor_enable(groundD, get_time_step());
  
  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1) {
    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */

    groundE_Valor = wb_distance_sensor_get_value(groundE);
    groundD_Valor = wb_distance_sensor_get_value(groundD);

    double ground_Sum = groundE_Valor + groundD_Valor;
 
    double left_ratio = groundE_Valor / ground_Sum;
    double right_ratio = 1 - left_ratio;
    
    left_ratio  = 2 * left_ratio - 1;
    right_ratio = 2 * right_ratio - 1;
    
    printf("left ratio: %.2f\n", left_ratio);
    
    
    // turn left
    if (left_ratio > 0.1) {
      Velocidades[LEFT]  = offset * right_ratio;
      Velocidades[RIGHT] = offset * left_ratio;
      speed_set = true;
      
      Velocidades[LEFT] = 0.0;
      Velocidades[RIGHT] += offset;
    }
    // turn right
    else if (right_ratio > 0.1){
      Velocidades[LEFT]  = offset * right_ratio;
      Velocidades[RIGHT] = offset * left_ratio;
      speed_set = true;
    
      Velocidades[LEFT] += offset;
      Velocidades[RIGHT] = 0.0;
    }
    else if (ground_Sum > 7.75) {
      Velocidades[LEFT]  = offset * right_ratio;
      Velocidades[RIGHT] = offset * left_ratio;
      speed_set = true;
      
      Velocidades[LEFT] += offset;
      Velocidades[RIGHT] += offset;    
    }
    else {
      // avoid adding to uninitialized speed
      if (!speed_set) {
        Velocidades[LEFT]  = 0.0;
        Velocidades[RIGHT] = 0.0;
        speed_set = true;
      }
      // always increase speed to widen curve radius
      Velocidades[LEFT] += offset;
      Velocidades[RIGHT] = 0.0;
    }
  
    /* Process sensor data here */

    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
    wb_motor_set_velocity(motorE, Velocidades[LEFT]);
    wb_motor_set_velocity(motorD, Velocidades[RIGHT]);
    
    robot_wait(0.25);
 
    printf("Velocidades:  %6.2f  %6.2f |",
        Velocidades[LEFT], Velocidades[RIGHT]); 
    printf("Sensores de Pista: Esq:%6.2f\tDir:%6.2f\n",
        groundE_Valor, groundD_Valor);
  }

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
