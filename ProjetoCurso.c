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
//static WbDeviceTag sensorDist[QtddSensorDist];
//static double sensorDist_Valor[QtddSensorDist];

static WbDeviceTag groundE, groundD;
static double groundE_Valor, groundD_Valor;

#define LEFT 0
#define RIGHT 1
#define MAX_SPEED 3.14
//static double weights[QtddSensorDist][2] = {{-1.3, -1.0},{-1.3, -1.0},{-0.5, 0.5},{0.0, 0.0},{0.0, 0.0},{0.05, -0.5},{-0.75, 0},{-0.75, 0}};

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
  int i,j;
  double Velocidades[2];
  
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
  
  /*
  //SENSORES
  sensorDist[0] = wb_robot_get_device("ps0");
  sensorDist[1] = wb_robot_get_device("ps1");
  sensorDist[2] = wb_robot_get_device("ps2");
  sensorDist[3] = wb_robot_get_device("ps3");
  sensorDist[4] = wb_robot_get_device("ps4");
  sensorDist[5] = wb_robot_get_device("ps5");
  sensorDist[6] = wb_robot_get_device("ps6");
  sensorDist[7] = wb_robot_get_device("ps7");
  
  for(i=0;i<QtddSensorDist;i++)
    wb_distance_sensor_enable(sensorDist[i], get_time_step());
  */  

  /*
  //POSIÇÃO ROBO
  //ATENÇÃO -> NÃO ESQUEÇA DE HABILITAR O SUPERVISOR *E* NOMEAR (DEF) O ROBO!
  WbNodeRef robot_node = wb_supervisor_node_get_from_def("e-Puck");
  if (robot_node == NULL) {
    fprintf(stderr, "No DEF ePuck node found in the current world file\n");
    exit(1);
  }
  WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation");
  const double *posicao_robo;
  */
  
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

    /*
    for (i = 0; i < QtddSensorDist; i++)
      sensorDist_Valor[i] = wb_distance_sensor_get_value(sensorDist[i])/4096;
    */

    /*    
    for (i = 0; i < 2; i++) {
      Velocidades[i] = 0.0;
      for (j = 0; j < QtddSensorDist; j++)
        Velocidades[i] += sensorDist_Valor[j] * weights[j][i];
   
      Velocidades[i] = offsets[i] + (Velocidades[i] * MAX_SPEED) +  ((double)rand()/RAND_MAX-.5);
      if (Velocidades[i] > MAX_SPEED)
        Velocidades[i] = MAX_SPEED;
      else if (Velocidades[i] < -MAX_SPEED)
        Velocidades[i] = -MAX_SPEED;
    }
    */

    groundE_Valor = wb_distance_sensor_get_value(groundE);
    groundD_Valor = wb_distance_sensor_get_value(groundD);

    double ground_Sum = groundE_Valor + groundD_Valor;
 
    double left_ratio = groundE_Valor / ground_Sum;
    double right_ratio = 1 - left_ratio;
    
    left_ratio  = 2 * left_ratio - 1;
    right_ratio = 2 * right_ratio - 1;
    
    printf("left ratio: %.2f\n", left_ratio);
    
    Velocidades[LEFT]  = offset * right_ratio;
    Velocidades[RIGHT] = offset * left_ratio;
    
    // turn left
    if (left_ratio > 0.1) {
      Velocidades[LEFT] = 0.0;
      Velocidades[RIGHT] += offset;
    }
    // turn right
    else if (right_ratio > 0.1){
      Velocidades[LEFT] += offset;
      Velocidades[RIGHT] = 0.0;
    }
    else {
      Velocidades[LEFT] += offset;
      Velocidades[RIGHT] += offset;    
    }
    /*
    // go backward
    else if (ground_Sum < 7.5) {
      double temp = Velocidades[LEFT];
      Velocidades[LEFT]  = -Velocidades[RIGHT];
      Velocidades[RIGHT] = -temp;
    }
    */
     
    /*
    for (i = 0; i < 2; i++) {
      Velocidades[i] = 0.0;
      for (j = 0; j < QtddSensorGround; j++)
        Velocidades[i] += sensorGround_Valor[j] * gWeights[j][i] / sensorGround_sum;
    
      //Velocidades[i] = offsets[i] + (Velocidades[i] * MAX_SPEED) +  ((double)rand()/RAND_MAX-.5);
      Velocidades[i] = Velocidades[i] * MAX_SPEED;
    
      if (Velocidades[i] > MAX_SPEED)
        Velocidades[i] = MAX_SPEED;
      else if (Velocidades[i] < -MAX_SPEED)
        Velocidades[i] = -MAX_SPEED;
    }
    */
  
    //posicao_robo = wb_supervisor_field_get_sf_vec3f(trans_field);
    
    
    /* Process sensor data here */

    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
    wb_motor_set_velocity(motorE, Velocidades[LEFT]);
    wb_motor_set_velocity(motorD, Velocidades[RIGHT]);
    
    robot_wait(0.25);
 
    //posicao_Caixa1 = wb_supervisor_field_get_sf_vec3f(Caixa1_field);
    printf("Velocidades:  %6.2f  %6.2f |",
        Velocidades[LEFT], Velocidades[RIGHT]); 
    //printf("Posição Robo: %6.2f  %6.2f |",posicao_robo[0], posicao_robo[2]);
    printf("Sensores de Pista: Esq:%6.2f\tDir:%6.2f\n",
        groundE_Valor, groundD_Valor);
  }

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
