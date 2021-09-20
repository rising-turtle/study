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

 

  

  err = stat(file_path, &st);

 

  if (!err) {

    printf("File exists.");

    }

  else {

    printf("File does not exist");

    return -1;

    }

    

  return 0;

}

 

int shell_find_file(char *file_name, char *file_path, char file_path_size) {

  // traverse the PATH environment variable to find the absolute path of a file/command

 

  char *f_name = strdup(file_name);

  char *dir_list = getenv("PATH");

  char *dir;

 

  while( (dir_list = strsep(&dir,":")) != NULL ) {

    char *f_path = strcat(dir, "/");

    f_path = strcat(f_path, f_name);

   

    if (shell_file_exists(f_path) == 0) {

      // file exists

      file_path = strdup(f_path);

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

      execv(file_path, argv); // Load the program

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

     printf("This far");

     while (*empty_test != '\0') {

       if (!isspace((unsigned char)*empty_test)) {

         // not empty

         empty = 0;

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

       while(isspace((unsigned char)*command)) command++;

       

       char *end = command + strlen(command) - 1;

       printf("end is %s\n", end);

       while(end > command && isspace((unsigned char)*end)) end--;

       printf("end is %s\n", end);

       end[1] = NULL; // '\0';

       

       

       if (strcmp(command, "exit") == 0) {

         // exit shell

         printf("Exiting Shell");

         exit = 1;

       }

       else {

         // regular command

         char *comm_name = strtok(command, " ");

         printf("comm_name is %s\n", comm_name);

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

           printf("after temp\n");
          

           for(int i = strlen(comm_name); i < strlen(command); i++){

           if(command[i] != ' '){

             // build arg

             temp += command[i];
             printf("tmp %s\n", temp); 

             }

           else{

             // arg built

             args[num_args] = temp;

             num_args ++;

             temp = "";

             }

           }

          

           char *ret_path;

           if (shell_file_exists(comm_name) == 0) {

             // exists in given dir

             shell_execute(comm_name, args);

           }

           else if (shell_find_file(comm_name, ret_path, 0) == 0) {

             // exists in PATH

            

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

