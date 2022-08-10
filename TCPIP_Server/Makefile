CC=g++
CPPFLAGS=-I/usr/include/mysql -I../Include
LDFLAGS=
LDLIBS=-lpthread -lmysqlclient
OBJS=db_mysql.o tcpip_packet.o client.o main.o
TARGET=TCPIP_Server
 
all: $(TARGET)
 
clean:
	rm -f *.o
	rm -f $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CPPFLAGS) -o $@ $(OBJS) $(LDLIBS)

