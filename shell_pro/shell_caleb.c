#include <stdio.h>

#include <string.h>

#include <stdlib.h>

#include <ctype.h>

#include <sys/wait.h>

#include <unistd.h>

#include <sys/stat.h>

 

/*

  Author:  Caleb Johnson

  Course:  COMP 340, Operating Systems

  Date:    17 September 2021

  Description:   This file implements the Shell program

  Compile with:  gcc -o shell shell.c

  Run with:      ./shell

 

*/

 

int shell_change_dir(char *dir_path);

int shell_file_exists (char *file_path);

int shell_find_file(char *file_name, char *file_path, char file_path_size);

int shell_execute(char *file_path, char **argv);

 

int shell_change_dir(char *dir_path) {

  //use the chdir() system call to change the current directory

 

  int exists = shell_file_exists(dir_path);

  if (exists == 0) {

    // exists

    chdir(dir_path);

   

  }

  else {

    // does not exist

    printf("Invalid path");

    return -1;

  }

 

  return 0;

 

}

 

int shell_file_exists (char *file_path) {

  // use the stat() system call to check if a file exists

  int err;
  struct stat st;
  printf("before stat\n");
  err = stat(file_path, &st);
  printf("after stat err = %d\n", err);
  if (!err) {
    printf("File exists.\n");

    }
  else {

    printf("File does not exist\n");

    return -1;

  }

  return 0;

}

 

int shell_find_file(char *file_name, char *file_path, char file_path_size) {

  // traverse the PATH environment variable to find the absolute path of a file/command

  printf("file_name %s file_path %s \n", file_name, file_path);

  char *f_name = strdup(file_name);

  char *dir_list = getenv("PATH");

  char *dir;

  printf("checkpoint 5  dir_list %s\n", dir_list);

  // while( (dir_list = strsep(&dir,":")) != NULL ) {
  while( (dir = strsep(&dir_list,":")) != NULL ) {

    // printf("checkpoint 6  dir_list %s dir %s \n", dir_list, dir);
    
    char * tmp_dir = strdup(dir); 
    char *f_path = strcat(tmp_dir, "/");
    f_path = strcat(f_path, f_name);
    printf(" f_path is %s\n", f_path);
    
    if (shell_file_exists(f_path) == 0) {

      // file exists
      printf("f_path is %s\n", f_path);
      // file_path = strdup(f_path);
      strcpy(file_path, f_path); 

      return 0;

    }

  }

 

  // no such file in PATH

  return -1;

}

 

 

int shell_execute(char *file_path, char **argv) {

  // execute the file with the command line arguments

 

  pid_t pid = fork();

 

               if(pid == 0){

 

                              // child process

    if (file_path[0] != '/') {

      char *local_cmd = strcat(file_path, "./");

      execv(local_cmd, argv);

    }

                              else {
          printf("file_path in shell_exec: %s argv: %s\n", file_path, argv);
          // execv(file_path, argv); // Load the program
          char * tmp_argv[1024] = {file_path, NULL};
          execv(file_path, tmp_argv); // Load the program

      }

 

                              // only print this if the program cannot be found

                              printf("I cannot find this program %s\n", file_path);

    return -1;

 

               }else{

                             

                              // parent process

                              sleep(1);

                              waitpid(pid, 0, 0); // wait for the child process to terminate

                              printf("Program %s has finished!\n", file_path); 

 

               }

               return 0;

}

 

 

int main (int argc, char *argv[]) {

   //run the shell

   printf("Shell running\n");

   printf("Enter Command: ");

   int exit = 0;

   while (exit == 0) {
     // 1.
     char line[1024];
     fgets(line, 1024, stdin);
     printf("line input: %s\n", line);
     char * empty_test = strdup(line);
     int empty = -1;

     // check whether it is a pure white space input 
     while (*empty_test != '\0') {
       if (!isspace((unsigned char)*empty_test)) {
         // not empty
         empty = 0;
         break; 
       }
       empty_test++;
     }

    

     if (empty == -1) {
       printf("Only Whitespace\n");
     }
     else {
       printf("This far line %s\n", line);
       // trim whitespace
       char *command = strdup(line);
       printf("command is %s\n", command); 

       // point to the first non-white_space charater '  ls -a'
       // e.g. '   ls -a', command point to 'l', end point to 'a'
       while(isspace((unsigned char)*command)) command++;
       
       char *end = command + strlen(command) - 1;

       // printf("point shift by %d\n", strlen(command) - 1); 
       // printf("command point to %c end point to %c\n", *command, *end);
       printf("end is %s\n", end);

       // point end to the last charater that is non_white_space 
       while(end > command && isspace((unsigned char)*end)) end--;

       printf("end points to %c\n", *end);

       end[1] = NULL; // '\0';

       if (strcmp(command, "exit") == 0) {
         // exit shell
         printf("Exiting Shell");
         exit = 1;
       }
       else {
         // regular command
         char *tmp_command = strdup(command);
         // printf("command before strtok %s\n", command);
         char *comm_name = strtok(tmp_command, " ");
         printf("comm_name is %s\n", comm_name);
         // printf("command after strtok %s\n", command); 

         if (!strcmp(comm_name, "cd")) {
           // change directory
           int i = 0;
           char new_dir[1024];
           while (i < strlen(command) - strlen(comm_name)) {
             new_dir[i] = command[strlen(comm_name)+i];
             i++;
           }
           new_dir[i] = '\0';
           shell_change_dir(new_dir);
         }
         else {

           // check if command is anywhere
           // parse args
           printf("before temp\n"); 
           char temp[1024] = "";
           char * args[1024];
           int num_args = 0;
           int tmp_ind = 0;
           printf("after temp\n");

           printf("comm_name: %s strlen(comm_name) is %d command is %s strlen(conmmand) %d\n", \ 
            comm_name, strlen(comm_name), command, strlen(command));
            
            // e.g. ls -a (i = 2; i< 5; i++)
           for(int i = strlen(comm_name)+1; i < strlen(command); i++){
              printf("here checkpoint 1\n");
              if(command[i] != ' '){

                // build arg
                // char *temp_square = strcat(temp, command[i]);
                printf("before strncat \n");
                strncat(temp, &command[i], 1);
                printf("here checkpoint 2 temp %s \n", temp);
             
                // strcpy(temp, temp_square);
                tmp_ind ++;
                printf("tmp %s\n", temp); 
              }else{ // if command[i] == ' '
                // arg built
                printf("checkpoint 4\n");
                args[num_args] = strdup(temp);
                num_args ++;
                strcpy(temp, NULL);
             }

           }

           char ret_path[1024] = "";
           printf("before shell_file_exists comm_name %s \n", comm_name);
           if (shell_file_exists(comm_name) == 0) {
             // exists in given dir
             printf("after shell_file_exists\n");
             shell_execute(comm_name, args);
           }

           else if (shell_find_file(comm_name, ret_path, 0) == 0) {

             // exists in PATH

             printf("comm_name: %s ret_path: %s args: %s\n", comm_name, ret_path, args);

             shell_execute(ret_path, args);

           }

           else {

             shell_execute(comm_name, args);

           }

          

         }

          

         }

        

       }

      

     }

    

   return 0;

}
