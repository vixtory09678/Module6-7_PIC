#ifndef PROTOCOL_H
#define PROTOCOL_H

#define BUFFER_SIZE 4
#define BUFFER_HEADER_SIZE 2

// read command
#define GO_HOME 0
#define X_POSITION 1
#define Y_POSITION 2
#define Z_POSITION 3
#define ROTATE 4
#define KEEP 5

// call response command
#define ERROR_DECODE 100
#define TASK_COMPLETE 101

typedef struct FeedBackProtocol
{
    BOOLEAN goHome;
    BOOLEAN xPosition;
    BOOLEAN yPosition;
    BOOLEAN zPosition;
    BOOLEAN rotation;
    BOOLEAN keeping;
} FeedProtocol;

FeedProtocol feedBack = {FALSE, FALSE, FALSE, FALSE, FALSE, FALSE};
FeedProtocol firstFeedBack = {FALSE, FALSE, FALSE, FALSE, FALSE, FALSE};

typedef struct DecodeData
{
    uint8_t cmd;
    int data1;
} decode_;

decode_ dataCom;

void sentData(uint8_t cmd, int data)
{
    register uint8_t dataByte[2];
    dataByte[0] = (data >> 8) & 0xFF;
    dataByte[1] = (data);

    int chksum = 0;
    for (int i = 0; i < 2; i++)
        chksum += dataByte[i];
    chksum = ~(chksum + cmd);

    printf("%c", 0xFF);
    printf("%c", 0xFF);
    printf("%c", cmd);
    printf("%c", dataByte[0]);
    printf("%c", dataByte[1]);
    printf("%c", (unsigned char)chksum);
}

decode_ decodePackage(uint8_t *data)
{
    decode_ code;
    code.cmd = ERROR_DECODE;

    //checksum
    int chksm = 0;
    for (int numIndex = 0; numIndex < BUFFER_SIZE - 1; numIndex++)
        chksm += data[numIndex] & 0xff;
    chksm = (~chksm) & 0xff;

    if (chksm != data[BUFFER_SIZE - 1])
        return code;

    // happy part
    code.cmd = data[0];
    code.data1 = (data[1] << 8) | data[2];
    return code;
}

int i = 0;
int countHeader = 0;
BOOLEAN isReady = false;
uint8_t data[BUFFER_SIZE];

#INT_RDA
void readSerialData()
{
    // receive data
    uint8_t d = getc();
    // putc(d);
    if (countHeader < 2)
    {
        if ((d & 0xff) == 0xFF)
            countHeader += 1;
        else
            countHeader = 0;
    }
    else
    {
        data[i++] = d;
        if (i > (BUFFER_SIZE - 1))
        {
            i = 0;
            countHeader = 0;
            isReady = true;
        }
    }
}

#endif