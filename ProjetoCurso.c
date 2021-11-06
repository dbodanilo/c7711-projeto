/*
 * File:          Controlador1.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
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
static WbDeviceTag sensorDist[QtddSensorDist];
static double sensorDist_Valor[QtddSensorDist];

static WbDeviceTag GroundE, GroundD, GroundC;
static double GroundE_Valor, GroundD_Valor, GroundC_Valor;


#define LEFT 0
#define RIGHT 1
#define MAX_SPEED 2
static double weights[QtddSensorDist][2] = {{-1.3, -1.0},{-1.3, -1.0},{-0.5, 0.5},{0.0, 0.0},{0.0, 0.0},{0.05, -0.5},{-0.75, 0},{-0.75, 0}};
static double offsets[2] = {0.5 * MAX_SPEED, 0.5 * MAX_SPEED};





/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
 
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
 

int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  int i,j;
  double Velocidades[2];
  
  
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
  
  //POSIÇÃO ROBO
  //ATENÇÃO -> NÃO ESQUEÇA DE HABILITAR O SUPERVISOR *E* NOMEAR (DEF) O ROBO!
  WbNodeRef robot_node = wb_supervisor_node_get_from_def("e-Puck");
  if (robot_node == NULL) {
    fprintf(stderr, "No DEF ePuck node found in the current world file\n");
    exit(1);
  }
  WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation");
  const double *posicao_robo;
  
  
  //sensor de solo  
  GroundD = wb_robot_get_device("IR0");
  wb_distance_sensor_enable(GroundD, get_time_step());
  
  GroundE = wb_robot_get_device("IR1");
  wb_distance_sensor_enable(GroundE, get_time_step());
  
  GroundC = wb_robot_get_device("IR2");
  wb_distance_sensor_enable(GroundC, get_time_step());

  
  
  
  
  
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

 for (i=0;i<QtddSensorDist;i++)
    sensorDist_Valor[i] = wb_distance_sensor_get_value(sensorDist[i])/4096;
     
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
    
    
    /* Process sensor data here */

    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
     posicao_robo = wb_supervisor_field_get_sf_vec3f(trans_field);
    
     GroundE_Valor = wb_distance_sensor_get_value(GroundE);
     GroundD_Valor = wb_distance_sensor_get_value(GroundD);
     GroundC_Valor = wb_distance_sensor_get_value(GroundC);
     
     bool lineE = GroundE_Valor > 1.2 * GroundD_Valor;
     bool lineD = GroundD_Valor > 1.2 * GroundE_Valor;
     //bool lineC = ???;
     
     if (lineD && !lineE) {
      // turn right
      Velocidades[RIGHT] = -Velocidades[RIGHT];
     }
     else if (lineE && !lineD) {
      // turn left
      Velocidades[LEFT] = -Velocidades[LEFT];
     }
     else {
       robot_wait(0.05);
     }
     
     /*
     // if on line: keep going
    if (GroundC_Valor > 2 && 
        GroundE_Valor < 5 && 
        GroundD_Valor < 5) {
      robot_wait(0.005);
    }
        
    // if line left of center: 
    else if (GroundE_Valor > GroundD_Valor) {
      // turn left
      Velocidades[LEFT] = -Velocidades[LEFT];
      // until line is centered
      while(GroundC_Valor < 2) {
        // actuate
        wb_motor_set_velocity(motorE, Velocidades[LEFT]);
        wb_motor_set_velocity(motorD, Velocidades[RIGHT]);
        robot_wait(0.005);
        GroundC_Valor = wb_distance_sensor_get_value(GroundC);        
      } 
    }
    // if line right of center
    else if (GroundD_Valor > GroundE_Valor) {
      // turn right
      Velocidades[RIGHT] = -Velocidades[RIGHT];
      // until line is centered
      while(GroundC_Valor < 2) {
        // actuate
        wb_motor_set_velocity(motorE, Velocidades[LEFT]);
        wb_motor_set_velocity(motorD, Velocidades[RIGHT]);
        robot_wait(0.005);
        GroundC_Valor = wb_distance_sensor_get_value(GroundC);        
      } 
    }
    */
    
    /*
    else {
      Velocidades[LEFT] = -Velocidades[LEFT];
      Velocidades[RIGHT] = -Velocidades[RIGHT];
    }
    */

    /*
    if (GroundD_Valor<5 || GroundE_Valor<5){
      if (GroundD_Valor<5) Velocidades[LEFT] = -Velocidades[LEFT]*1;
      else if (GroundE_Valor<5) Velocidades[RIGHT] = -Velocidades[RIGHT]*1;
    }
    */

    // actuate
    wb_motor_set_velocity(motorE, Velocidades[LEFT]);
    wb_motor_set_velocity(motorD, Velocidades[RIGHT]);
 
    //posicao_Caixa1 = wb_supervisor_field_get_sf_vec3f(Caixa1_field);
    printf("Velocidades:  %6.2f  %6.2f |",Velocidades[LEFT],Velocidades[RIGHT]); 
    printf("Posição Robo: %6.2f  %6.2f |",posicao_robo[0], posicao_robo[2]);
    printf("Sensores de Pista: Esq:%6.2f  Cen:%6.2f Dir:%6.2f\n",GroundE_Valor, GroundC_Valor, GroundD_Valor);
    
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
