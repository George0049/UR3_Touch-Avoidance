#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/wait.h>
#include <signal.h>
#include <math.h>

#define PORT "5556"  // the port users will be connecting to

#define BACKLOG 10	 // how many pending connections queue will hold
#define MAXDATASIZE 100

void sigchld_handler(int s)
{
	// waitpid() might overwrite errno, so we save and restore it:
	int saved_errno = errno;

	while(waitpid(-1, NULL, WNOHANG) > 0);

	errno = saved_errno;
}


// get sockaddr, IPv4 or IPv6:
void *get_in_addr(struct sockaddr *sa)
{
	if (sa->sa_family == AF_INET) {
		return &(((struct sockaddr_in*)sa)->sin_addr);
	}

	return &(((struct sockaddr_in6*)sa)->sin6_addr);
}


void setPoint(char* buf, geometry_msgs::Pose* pose)
{
	std::string str(buf);
	std::vector<std::string> p;
	int po[7];
	po[0] = str.find("[")+1;
	po[6] = str.find("]")+1;

	for(int i=0; i<6; i++) {
		po[i+1] = str.find(",",po[i])+1;
		p.push_back(str.substr(po[i],po[i+1]-po[i]-1));
	}

	pose->position.x = std::atof(p[3].c_str())/1000;
	pose->position.y = std::atof(p[4].c_str())/1000;
	pose->position.z = std::atof(p[5].c_str())/1000;

	float cx = cos(std::atof(p[0].c_str())/2);
	float sx = sin(std::atof(p[0].c_str())/2);
	float cy = cos(std::atof(p[1].c_str())/2);
	float sy = sin(std::atof(p[1].c_str())/2);
	float cz = cos(std::atof(p[2].c_str())/2);
	float sz = sin(std::atof(p[2].c_str())/2);

	pose->orientation.w = cx*cy*cz + sx*sy*sz;
	pose->orientation.x = sx*cy*cz + cx*sy*sz;
	pose->orientation.y = cx*sy*cz + sx*cy*sz;
	pose->orientation.z = cx*cy*sz + sx*sy*cz;

//	std::cout<<cx<<":"<<sx<<":"<<cy<<":"<<sy<<":"<<cz<<":"<<sz<<std::endl;
	std::cout<<pose->position.x<<pose->position.y<<pose->position.z<<pose->orientation.w<<pose->orientation.x<<pose->orientation.y<<pose->orientation.z<<std::endl;
}

int main(int argc, char** argv)
{
	int sockfd, new_fd;  // listen on sock_fd, new connection on new_fd
	struct addrinfo hints, *servinfo, *p;
	struct sockaddr_storage their_addr; // connector's address information
	socklen_t sin_size;
	struct sigaction sa;
	int yes=1;
	char s[INET6_ADDRSTRLEN];
	int rv;


	memset(&hints, 0, sizeof hints);
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_flags = AI_PASSIVE; // use my IP

	if ((rv = getaddrinfo(NULL, PORT, &hints, &servinfo)) != 0) {
		fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
		return 1;
	}

	// loop through all the results and bind to the first we can
	for(p = servinfo; p != NULL; p = p->ai_next) {
		if ((sockfd = socket(p->ai_family, p->ai_socktype,
				p->ai_protocol)) == -1) {
			perror("server: socket");
			continue;
		}

		if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &yes,
				sizeof(int)) == -1) {
			perror("setsockopt");
			exit(1);
		}

		if (bind(sockfd, p->ai_addr, p->ai_addrlen) == -1) {
			close(sockfd);
			perror("server: bind");
			continue;
		}

		break;
	}

	freeaddrinfo(servinfo); // all done with this structure

	if (p == NULL)  {
		fprintf(stderr, "server: failed to bind\n");
		exit(1);
	}

	if (listen(sockfd, BACKLOG) == -1) {
		perror("listen");
		exit(1);
	}

	sa.sa_handler = sigchld_handler; // reap all dead processes
	sigemptyset(&sa.sa_mask);
	sa.sa_flags = SA_RESTART;
	if (sigaction(SIGCHLD, &sa, NULL) == -1) {
		perror("sigaction");
		exit(1);
	}

	printf("server: waiting for connections...\n");

	int numbytes;
	char buf[MAXDATASIZE];

	while(1)  {  // main accept() loop

		sin_size = sizeof their_addr;
		new_fd = accept(sockfd, (struct sockaddr *)&their_addr, &sin_size);
		if (new_fd == -1) {
			perror("accept");
			continue;
		}

		inet_ntop(their_addr.ss_family,
			get_in_addr((struct sockaddr *)&their_addr),
			s, sizeof s);
		printf("server: got connection from %s\n", s);

		if (!fork()) { // this is the child process
			close(sockfd); // child doesn't need the listener
			if (send(new_fd, "Hello, world!", 13, 0) == -1)
				perror("send");
			
			//初始化ROS，用fork开辟子进程，前面不能有ros的东西
				ros::init(argc, argv, "test_tcp_node");
				ros::NodeHandle node_handle;

				ros::Publisher pub = node_handle.advertise<geometry_msgs::Pose>("ur/tcp",100,true);
				ros::Rate rate(1);

				geometry_msgs::Pose msg;

			//接收数据处理函数
			while(ros::ok()) {
				if ((numbytes = recv(new_fd, buf, MAXDATASIZE-1, 0)) == -1) {
						perror("recv");
						exit(1);
				}
				buf[numbytes] = '\0';
				std::cout<<numbytes<<std::endl;

				if(numbytes == 0) {
					close(new_fd);
					exit(0);
				}

				if(buf[0] == 'm') {
					setPoint(buf, &msg);
					std::cout<<"get one point"<<std::endl;

					pub.publish(msg);
					rate.sleep();
				}

			}
			close(new_fd);
			exit(0);
		}
		close(new_fd);  // parent doesn't need this
		exit(0);
	}
	exit(0);
	return 0;
}

