#include <unistd.h>
#include <sys/types.h>
#include <signal.h>

#include <stdio.h>


// child process send a signal to parent process

int g_syn = false;

void getsignal(int ){
	g_syn = true;
	printf("process: %d get this signal!\n",getpid());
}

int main()
{
	if(signal(SIGUSR1,getsignal)==SIG_ERR){
		printf("cannot catch singnal SIGUSR1\n");
		return -1;
	}
	
	pid_t pid = getpid();
	printf("my process pid is: %d!\n",pid);
	pid = fork();	// create child process
	if(pid == 0){
		printf("I am child process! my pid is %d\n",getpid());
		printf("sleep 5 seconds!\n");
		sleep(5);
		kill(getppid(),SIGUSR1);
		_exit(0);
	}
	else if(pid>0){
		printf("I am parent process! my pid is %d\n",getpid());
	}
	else{
		printf("failed to create process!\n");
	}
	
	while(!g_syn){
		pause(); // suspend until another signal arrives
	}

	printf("Parent process will exit!\n");
	
	return 0;
}
