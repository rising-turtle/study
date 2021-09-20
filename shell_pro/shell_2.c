#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <limits.h>


int shell_change_dir(char *dir_path) {
  // use the chdir() system call to change the current directory
  return chdir(dir_path);
}


int shell_file_exists(char *file_path) {
  // use the stat() system call to check if a file exists
  struct stat stats;
  return stat(file_path, &stats);
}


int shell_find_file(char *file_name, char *file_path, char file_path_size) {
  // traverse the PATH environment variable to find the absolute path of a file/command
  char * envPATH = getenv("PATH");
  char * dir;
  
  while ((dir = strsep(&envPATH, ":")) != NULL) {
    sprintf(file_path, "%s/%s", dir, file_name);
    if(shell_file_exists(file_path)) { 
      return -1;
    }
  }
  return 0;
}

int shell_execute(char *file_path, char **argv) {
  // execute the file with the command line arguments
  pid_t pid;
  pid = fork();
  if (pid<0) {
    return -1;
  }
  else if (pid == 0) {
    execv(file_path, argv);
    printf("File not found/n");
  }
  else {
    wait(NULL);
  }
  return 0;
}


int main (int argc, char *argv[]) {
   //run the shell
   char * usr = getlogin();
   char host[HOST_NAME_MAX + 1];
   gethostname(host, sizeof(host));
   char cwd[4096];
   int isRunning = 1;
   
   // char * path; 
   char path[1024]; 
   char * cmd;
   char cmdLine[4096];
   char * cmdArgs[4096];
   int i = 0;
   
   do {
     
     // Get current working directory
     getcwd(cwd, 4096);
     char* temp = strtok(cwd, "/");
     char* curFile;
     
     while(temp != NULL) {
       curFile = temp;
       temp = strtok(NULL, "/");
     }
     
     printf("%s@%s:myShell:~/%s$ ", usr, host, curFile);
     fgets(cmdLine, sizeof(cmdLine), stdin);
     
     char * cmdLinePtr = cmdLine;

     while ((cmd = strsep(&cmdLinePtr, " \n")) != NULL) {
       cmdArgs[i] = cmd;
       i++;
     }
     
     // 4 parse cmd line
     if (!strcmp(cmdArgs[0],"exit")) {
       printf("\nmyShell Shutting Down\n\n");
       isRunning = 0;
     }
     else if (!strcmp(cmdArgs[0],"cd")) {
       shell_change_dir(cmdArgs[1]);
     }
     else {
       if (cmdArgs[0][0] == '/' && !shell_file_exists(cmdArgs[0])) {
         shell_execute(cmdArgs[0], cmdArgs);
       }
       else {
         printf("0 cmdArgs[0]: %s\n",cmdArgs[0]); // TEST
         sprintf(path, "./%s", cmdArgs[0]);
         printf("1 cmdArgs[0]: %s\n",cmdArgs[0]); // TEST
         if (!shell_file_exists(path)) {
           cmdArgs[0] = path;
           shell_execute(cmdArgs[0], cmdArgs);
         }
         else {
           printf("2 cmdArgs[0]: %s\n",cmdArgs[0]); // TEST
           if(!shell_find_file(cmdArgs[0],path,127)) {
             cmdArgs[0] = path;
             shell_execute(cmdArgs[0],cmdArgs);
           }
         }
       }
       printf("Error occured");
     }
   } while (isRunning);
   return 0;
   /*
   while (!exit)
   {
	1. display prompt and wait for user input
		// generate some prompt 
		// e.g. username@hostname:~/david$  
	
	2. filter out whitespace command 
	
	3. if command line contains non-whitespace characters
	
	4. parse command line
		if the specified command is “exit”, terminate the program taking care to release any allocated resources.
		if the specified command is “cd”
			change the current working directory to the specified directory path using shell_change_dir()
		if the command is specified using an absolute path, exists in the user’s PATH or exists in the current folder
			execute the command using shell_execute()
		else
			report an error message
    }
   */
}