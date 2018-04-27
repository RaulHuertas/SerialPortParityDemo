#include <QCoreApplication>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <signal.h>
using namespace std;


volatile sig_atomic_t terminar = 0;
void signal_handler(int signal)
{
    terminar = 1;
}

unsigned char transmitBuffer[2048];
std::vector<unsigned char> readBuffer;


void serPortParityBit(int puerto_fd, int bitParidad){
    struct termios configPort;
    memset(&configPort, 0, sizeof(configPort));
    tcgetattr(puerto_fd, &configPort);
    if(bitParidad==0){
        configPort.c_cflag |= (PARENB | CMSPAR);
        configPort.c_cflag &= ~PARODD;
    }else{
        configPort.c_cflag |= (PARENB | CMSPAR | PARODD);
    }
    if ( tcsetattr(puerto_fd, TCSANOW, &configPort) < 0) {
        cout<<"No se pudo configurar el puerto"<<endl;
    }
}

void serPortParity_Odd(int puerto_fd){
    struct termios configPort;
    memset(&configPort, 0, sizeof(configPort));
    tcgetattr(puerto_fd, &configPort);
    configPort.c_cflag |= (PARENB);
    configPort.c_cflag &= ~CMSPAR;
    configPort.c_cflag |= PARODD;
    if ( tcsetattr(puerto_fd, TCSANOW, &configPort) < 0) {
        cout<<"No se pudo configurar el puerto"<<endl;
    }
}

void serPortParity_Even(int puerto_fd){
    struct termios configPort;
    memset(&configPort, 0, sizeof(configPort));
    tcgetattr(puerto_fd, &configPort);
    configPort.c_cflag |= (PARENB);
    configPort.c_cflag &= ~CMSPAR;
    configPort.c_cflag &= ~PARODD;
    if (tcsetattr(puerto_fd, TCSANOW, &configPort) < 0) {
        cout<<"No se pudo configurar el puerto"<<endl;
    }
}


int enviarGP(int puerto_fd,int address ){
    transmitBuffer[0] = (address|0x80U);
    memset(&transmitBuffer[0],0xFF,sizeof(transmitBuffer));

//    write(puerto_fd, &transmitBuffer[0], 1);
    serPortParityBit(puerto_fd,1);
    write(puerto_fd, &transmitBuffer[0], 1);
    //usleep(1000);
    tcflush(puerto_fd,TCOFLUSH );
    serPortParityBit(puerto_fd,0);
    write(puerto_fd, &transmitBuffer[0], 1);
    //usleep(2000);
    tcflush(puerto_fd, TCOFLUSH);
//    write(puerto_fd, &transmitBuffer[0], 1);
//    usleep(1000);

//    int rcpt = read(puerto_fd, &readBuffer[0], 1);
//    if(rcpt<0){
//        return rcpt;
//    }
//    if(rcpt==0){
//        return -1;
//    }
//    if(rcpt>1){
//        return -1;
//    }
    return readBuffer[0];
}

int main(int argc, char *argv[])
{
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    signal(SIGKILL, signal_handler);
    readBuffer.resize(128);
    QCoreApplication a(argc, argv);
    if(argc<3){
        cout<<"Invoauqe el programa con los siguietnes argumentos: <puerto> <direcion Maquina SAS>"<<endl;
        return 0;
    }
    #ifdef __ARM_ARCH
        cout<<"PROCESADOR NANOPI"<<endl;
        //int serial_fd = open("/dev/ttyS1",   O_RDWR );
    #else
        cout<<"PROCESADOR INTEL"<<endl;
        //int serial_fd = open("/dev/ttyUSB0",   O_RDWR );
    #endif //__i386__
    int serial_fd = open(argv[1],   O_RDWR );
    int direccionSAS = atoi(argv[2]);

    if(serial_fd<=0){
        cout<<"No se pudo abrir el puerto"<<endl;
        return -1;
    }
    struct termios configPort;
    memset(&configPort, 0, sizeof(configPort));
    tcgetattr(serial_fd, &configPort);
    cfsetospeed(&configPort, B9600);
    configPort.c_lflag &= ~ICANON;//elimina deteccio nde caracteres especiales
    configPort.c_lflag |= PARENB | CMSPAR;//elimina la autogeneracio nde paridad
    configPort.c_lflag |= PARODD;//Habilitar "sticky parity", iniciado en 0
    configPort.c_lflag &= ~CSTOPB;//Habilitar "sticky parity", iniciado en 0
    configPort.c_cc[VMIN] = 0;
    configPort.c_cc[VTIME] = 2;
    if (tcsetattr(serial_fd, TCSANOW, &configPort) < 0) {
        close(serial_fd);
        cout<<"No se pudo configurar el puerto"<<endl;
        return -1;
    }

    //POLL A SINGLE EVENT
    while(terminar==0){
//        usleep(200*1000);
        int poll = enviarGP(serial_fd, direccionSAS);
//        if(poll<=0){
//            cout<<"No hay respuesta de GP"<<endl;
//            continue;
//        }
//        cout<<"Excepcion recibida "<<std::hex<<poll<<endl;
//        cout<<std::dec;
    }





    close(serial_fd);




    return 0;
}
