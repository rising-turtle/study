#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <limits.h>


int shell_change_dir(char* dir_path) {
    // use the chdir() system call to change the current directory
    return chdir(dir_path);
}


int shell_file_exists(char* file_path) {
    // use the stat() system call to check if a file exists
    struct stat stats;
    return stat(file_path, &stats);
}


int shell_find_file(char* file_name, char* file_path, char file_path_size) {
    // traverse the PATH environment variable to find the absolute path of a file/command
    char* envPATH = getenv("PATH");
    char* dir;

    while ((dir = strsep(&envPATH, ":")) != NULL) {
        dir = strcat(dir, "/");
        dir = strcat(dir, file_name); // sprintf() INSTEAD
        if (shell_file_exists(dir)) {
            file_path = strdup(dir);
            return 0;
        }
    }
    return -1;
}

int shell_execute(char* file_path, char** argv) {
    // execute the file with the command line arguments
    pid_t pid;
    pid = fork();
    if (pid < 0) {
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


int main(int argc, char* argv[]) {
    //run the shell
    char* usr = getlogin();
  
    char* cmd;
    char cmdLine[4096];
    char* cmdArgs[4096];
    int i = 0;
    int isRunning = 1; 

    do {
        char host[HOST_NAME_MAX + 1];
        gethostname(host, sizeof(host));
        //Get current working directory
    char cwd[200];
    getcwd(cwd, 200);

    char* temp = strtok(cwd, "/");
    char* curFile;

    while (temp != NULL) {
        curFile = temp;
        temp = strtok(NULL, "/");
    }


        printf("%s@%s:myShell:%s$ ", usr, host, curFile);
        fgets(cmdLine, sizeof(cmdLine), stdin);

        char* cmdLinePtr = cmdLine;

        printf("cmdline: %s\n", cmdLine); 
        while ((cmd = strsep(&cmdLinePtr, " \n")) != NULL) {
            cmdArgs[i] = cmd;
            printf("param %d is %s\n", i, cmd);
            i++;
        }

        printf("sizeof(argv[1]) is %d\n", strlen(cmdArgs[1])); 

        // 4 parse cmd line
        if (!strcmp(cmdArgs[0], "exit")) {
            isRunning = 0;
            printf("\nmyShell Shutting Down\n\n");
        }
        else if (!strcmp(cmdArgs[0], "cd")) {
            printf("Now I am here cmdArgs[0] %s cmdArgs[1] is %s\n", cmdArgs[0], cmdArgs[1]);
            cmdArgs[1] = "..";
            int tmp = shell_change_dir(cmdArgs[1]);
            printf("tmp = %d\n", tmp); 
        }
        else {
            if (cmdArgs[0][0] == '/' && shell_file_exists(cmdArgs[0])) {
                shell_execute(cmdArgs[0], cmdArgs);
            }
            else {
                char* path;
                path = strcat("./", cmdArgs[0]);
                if (shell_file_exists(path)) {
                    cmdArgs[0] = path;
                    shell_execute(cmdArgs[0], cmdArgs);
                }
                else {
                    if (shell_find_file(cmdArgs[0], path, 127)) {
                        cmdArgs[0] = path;
                        shell_execute(cmdArgs[0], cmdArgs);
                    }
                }
            }
            printf("Error occured");
        }
       printf("I am here for next run \n");
    } while(isRunning);// while (!strcmp(cmd, "exit"));
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