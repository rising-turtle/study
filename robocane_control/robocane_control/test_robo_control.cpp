
/*
 * Oct. 12, 2016 David Z 
 *
 * A test file for robocane_control 
 *
 *
 * */

#include "robocane_control.h"
#include <ros/ros.h>
#include <stdio.h>
#include <string.h>

void test(); 

void exp1(int argc, char* argv[]);
void exp2(); 

using namespace std; 

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "test_robo_control"); 
  ros::NodeHandle n; 

  // test(); 
  exp1(argc, argv); 
  
  return 0; 
}

void exp1(int argc, char* argv[])
{
  CRoboCane rc; 
  if(!rc.init())
  {
    // ROS_INFO("init Robocane-control failed, return "); 
    printf("init Robocane-control failed return"); 
    return ;
  }
  
  char cmd[128] = {0}; 
  char mode; 
  int value = 90000; 
  int default_loop = 10; 
  if(argc > 1) default_loop = atoi(argv[1]); 

  cout <<" test_robo_control.cpp: swingPositionLoop = " <<default_loop<<endl;

  // while(ros::ok())
  if(!rc.startDumpEncoder("encoder_output.log", 500))
  {
    cout <<" what? failed to start dump encoder? "<<endl;
  }
  while(ros::ok())
  {
    sleep(3); 
    // rc.swingVelocityMode(value, default_loop); 
    rc.swingPositionMode(value, default_loop);
    // rc.movePosition(value, 100);
    sleep(1); 
    rc.uninit(); 
    break; 
  }
  sleep(1); 
  
  ROS_INFO("finsih exp1 ");
  return ;
}

void exp2(){}

void test()
{
  CRoboCane rc; 
  if(!rc.init())
  {
    // ROS_INFO("init Robocane-control failed, return "); 
    printf("init Robocane-control failed return"); 
    return ;
  }
  
  char cmd[128] = {0}; 
  char mode; 
  int value; 
  int default_loop = 2; 
  // while(ros::ok())
  if(!rc.startDumpEncoder("encoder_output.log", 300))
  {
    cout <<" what? failed to start dump encoder? "<<endl;
  }
  while(ros::ok())
  {
    printf("input cmd: [swing|move|quit] [v|p] value\n");
    scanf("%s %c %d", cmd, &mode, &value); 
    
    cout << cmd <<" : "<<mode<<" : "<<value<<endl;

    if(strcmp(cmd, "swing") == 0) // want swing 
    {
      if(mode == 'v')
      {
        printf(" swingVelocityMode with vel= %d \n", value); 
        rc.swingVelocityMode(value, default_loop); 
      }else{ // otherwise assume it is p
        printf(" swingPositionMode with pos= %d \n", value); 
        rc.swingPositionMode(value, default_loop); 
      }
    }else if(strcmp(cmd, "move") == 0)
    {
      if(mode == 'v')
      {
        printf( "moveVelocity with vel= %d \n", value); 
        rc.moveVelocity(value); 
      }else{ // otherwise assume it is p
        printf( "movePosition with pos= %d \n", value); 
        rc.movePosition(value); 
      }
    }else if(strcmp(cmd, "halt") == 0)
    {
      // TODO: 
      /*
      if(mode == 'v')
      {
        printf(" haltVelMove \n"); 
        rc.haltVelMove(); 
      }else{ // otherwise assume it is p
        printf(" haltPosMove \n");
        rc.haltPosMove(); 
      }*/
      
    }else if(strcmp(cmd, "quit") == 0)
    {
        rc.uninit(); 
        break; 
    }else{
    
      cout << "unknown cmd "<<cmd<<endl;
      // break; 
    }
    usleep(10); 
  }

  return ;
}








