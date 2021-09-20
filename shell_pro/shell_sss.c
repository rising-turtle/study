/**
*  Author:  Nathan Fulton and Carly Williams
*  Course:  COMP 340 A, Operating Systems
*  Date:    16 October 2020
*  Assignment:    Project 1
*  Description:   This file implements the Shell program
*  Compile with:  gcc shell.c -o shell
*  Run with:      ./shell
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <ctype.h>

#define MAX_COMMAND_SIZE 255
#define PATH_MAX 255

/**
 * Changes the current working directory of the program
 * 
 * dir_path: directory to change to
 */
int shell_change_dir(char *dir_path) {
    return chdir(dir_path);
}

/**
 * Determines if file_path exists on the local filesystem.
 * Returns 0 if the file exists, otherwise -1.
 * 
 * file_path: file to check if exists
 */
int shell_file_exists(char *file_path) {
    // use the stat() system call to check if a file exists
    struct stat buffer;
    return stat(file_path, &buffer);
}

/**
 * Determines if file_name exists on the PATH. 
 * On success puts the absolute path for file_name
 * into file_path and returns 0. Otherwise returns -1.
 * 
 * file_name: name of file to search for in PATH directories
 * file_path: buffer for the absolute file path to be put in if found
 * file_path_size: size of file_path, needs to be large enough to hold
 *                 the resulting absolute path
 */
int shell_find_file(char *file_name, char *file_path, char file_path_size) {

    // get the PATH enviornment variable
    char *get_env_path = getenv("PATH");
    if (get_env_path == NULL) {
        return -1;
    }

    // strsep will modify the env_path pointer so we need to keep 
    // env_path_original to point to the memory from strdup that
    // we'll need to free later
    char *env_path, *env_path_original;
    env_path = env_path_original = strdup(get_env_path);

    int found = -1;
    char *token;
    while (found != 0 && (token = strsep(&env_path, ":")) != NULL) {
        snprintf(file_path, file_path_size, "%s/%s", token, file_name);
        found = shell_file_exists(file_path);
    }

    free(env_path_original); // free memory from strdup
    return found;
}

/**
* Takes an absolute file path and and array of command line arguments
* attempts to execute file with the arguments
* returns 0 if succesful or -1 if unsuccessful
*
*file_path: name of the file being executed
*argv: array of command line arguments
*/
int shell_execute(char *file_path, char **argv) {
    // execute the file with the command line arguments

    pid_t pid;//ids of the processes

    pid = fork();//creates the child process 

    //checks for error in forking
    if (pid < 0) {
        return -1;
    }

    //code executed by child process
    if (pid == 0) {
        execv(file_path, argv);
        printf("%s: encountered an error\n", file_path);
        //exits and returns -1 if execv has an error
        exit(-1);
    }//if

    //parent waits for child process to be joined
    wait(NULL);
    return 0;
}

/**
 * Takes a null-terminated c-string and returns 1 if it contains
 * any non-whitespace characters, otherwise returns 0
 * 
 * str: c-string to check if contains non-whitespace
 */
int is_only_whitespace(char *str) {
    while (*str != '\0') {
        if (!isspace((unsigned char)*str)) {
            return 1;
        }
        str++;
    }
    return 0;
}

/**
 * Runs the shell main loop
 * Supports cd, exit, and executing files given an absolute path
 * or if they exist on the PATH or in the current directroy
 */
int main (int argc, char *argv[]) {
    int exit = 0; // condition for main shell loop
    char *user = getenv("USER");
    char *hostname = getenv("HOSTNAME");
    char cwd[PATH_MAX]; // current working directory
    char inputBuffer[MAX_COMMAND_SIZE]; // user input buffer
    char *ptrC; // char pointer for traversing input buffer
    char *command; // inputted command
    char *arguments[MAX_COMMAND_SIZE]; // inputted arguments
    char file_path[PATH_MAX]; // absolute path for command
    int i; // loop control variable

    while (!exit) { // shell main loop

        // get the current working directory
        getcwd(cwd, sizeof(cwd));

        // print the shell prompt
        printf("%s@%s:%s$ ", user != NULL ? user : "user", hostname != NULL ? hostname : "hostname", cwd);

        // get user input
        fgets(inputBuffer, MAX_COMMAND_SIZE, stdin);

        // ignore all whitespace inputs
        if (is_only_whitespace(inputBuffer) != 0) {   

            // clear any previous arguments
            for (i = 0; i < MAX_COMMAND_SIZE; i++) {
                arguments[i] = NULL;
            }

            // clear any previous file_path
            file_path[0] = '\0';

            // get the command from the input
            char *ptrC = strtok(inputBuffer, " \n");
            command = ptrC;

            // by convention the first argument is the command
            arguments[0] = command;

            // get the arguments
            i = 1;
            while ((ptrC = strtok(NULL, " \n")) != NULL) {
                arguments[i] = ptrC;
                i++;
            }

            if (command != NULL) {
                if (strcmp(command, "exit") == 0) { // exit command
                    exit = 1;
                    break;
                } else if (strcmp(command, "cd") == 0) { // cd command
                    
                    printf("sizeof(argv[1]) is %d\n", strlen(arguments[1])); 

                    // cd command needs an argument dir to change to
                    if (arguments[1] != NULL) {
                        printf("cd arguments[1] = %s\n", arguments[1]);
                        arguments[1] = "..";
                        if (shell_change_dir(arguments[1]) != 0) {
                            printf("cd: Error changing directory to %s\n", arguments[1]);
                        }
                    } else {
                        printf("cd: No directory provided.\n");
                    }
                } else { // all other commands

                    // run file if it is an absolute path exists or exists in cwd,
                    // otherwise check if it exists on PATH and run it if it does
                    if (shell_file_exists(command) == 0) {
                        if (shell_execute(command, arguments) != 0) {
                            printf("%s: encountered an error\n", command);
                        }
                    } else if (command[0] != '/' && shell_find_file(command, file_path, PATH_MAX) == 0) {
                        if (shell_execute(file_path, arguments) != 0) {
                            printf("%s: encountered an error\n", file_path);
                        }
                    } else {
                        printf("%s: command not found\n", command);
                    }
                }
            } else {
                printf("Error parsing input.\n");
            }
        }
    }

    return 0;
}
