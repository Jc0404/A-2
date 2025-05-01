#include <stdlib.h>
#include <stdio.h>
#include "emulator.h"
#include "sr.h"

#define bool int
#define true 1
#define false 0

/* ******************************************************************
   Go Back N protocol.  Adapted from J.F.Kurose
   ALTERNATING BIT AND GO-BACK-N NETWORK EMULATOR: VERSION 1.2

   Network properties:
   - one way network delay averages five time units (longer if there
   are other messages in the channel for GBN), but can be larger
   - packets can be corrupted (either the header or the data portion)
   or lost, according to user-defined probabilities
   - packets will be delivered in the order in which they were sent
   (although some can be lost).

   Modifications:
   - removed bidirectional GBN code and other code not used by prac.
   - fixed C style to adhere to current programming style
   - added GBN implementation
**********************************************************************/

#define RTT 16.0                  /* round trip time.  MUST BE SET TO 16.0 when submitting assignment */
#define WINDOWSIZE 6              /* the maximum number of buffered unacked packet */
#define SEQSPACE (WINDOWSIZE * 2) /* the min sequence space for GBN must be at least windowsize * 2 */
#define NOTINUSE (-1)             /* used to fill header fields that are not being used */
#define TICK 1.0                  /* heart beat to check whether send or not*/

/* generic procedure to compute the checksum of a packet.  Used by both sender and receiver
   the simulator will overwrite part of your packet with 'z's.  It will not overwrite your
   original checksum.  This procedure must generate a different checksum to the original if
   the packet is corrupted.
*/
int ComputeChecksum(struct pkt packet)
{
  int checksum = 0;
  int i;

  checksum = packet.seqnum;
  checksum += packet.acknum;
  for (i = 0; i < 20; i++)
    checksum += (int)(packet.payload[i]);

  return checksum;
}

bool IsCorrupted(struct pkt packet)
{
  if (packet.checksum == ComputeChecksum(packet))
    return (false);
  else
    return (true);
}

/********* Sender (A) variables and functions ************/

static struct pkt sendbuffer[SEQSPACE]; /* array for storing packets waiting for ACK */
bool acked[SEQSPACE];                   /* array to store the acked packet*/
static double sendtime[SEQSPACE];       /* time when each packet was last sent*/
static int A_windowfirst;               /* array indexes of the first packet awaiting ACK */
static int A_windowcount;               /* the number of packets currently awaiting an ACK */
static int A_nextseqnum;                /* the next sequence number to be used by the sender */
static int time_flag;                   /* marks if the timer is running */

/* called from layer 5 (application layer), passed the message to be sent to other side */
void A_output(struct msg message)
{
  struct pkt sendpkt;
  int i;
  /* if not blocked waiting on ACK */
  if (A_windowcount < WINDOWSIZE)
  {
    if (TRACE > 1)
      printf("----A: New message arrives, send window is not full, send new messge to layer3!\n");

    /* create packet */
    sendpkt.seqnum = A_nextseqnum;
    sendpkt.acknum = NOTINUSE;
    for (i = 0; i < 20; i++)
      sendpkt.payload[i] = message.data[i];
    sendpkt.checksum = ComputeChecksum(sendpkt);

    /* put packet in window buffer */
    /* windowlast will always be 0 for alternating bit; but not for GoBackN */

    sendbuffer[A_nextseqnum] = sendpkt;
    acked[A_nextseqnum] = false;
    sendtime[A_nextseqnum] = RTT;
    A_windowcount++;

    /* send out packet */
    if (TRACE > 0)
      printf("Sending packet %d to layer 3\n", sendpkt.seqnum);
    tolayer3(A, sendpkt);

    /* start timer if first packet in window */
    if (!time_flag)
    {
      starttimer(A, TICK);
      time_flag = true;
    }

    /* get next sequence number, wrap back to 0 */
    A_nextseqnum = (A_nextseqnum + 1) % SEQSPACE;
  }
  /* if blocked,  window is full */
  else
  {
    if (TRACE > 0)
      printf("----A: New message arrives, send window is full\n");
    window_full++;
  }
}

/* called from layer 3, when a packet arrives for layer 4
   In this practical this will always be an ACK as B never sends data.
*/
void A_input(struct pkt packet)
{
  int ack, pos;
  /* if received ACK is not corrupted */
  if (!IsCorrupted(packet))
  {
    if (TRACE > 0)
      printf("----A: uncorrupted ACK %d is received\n", packet.acknum);
    total_ACKs_received++;

    ack = packet.acknum;

    /* check if new ACK or duplicate */
    pos = (ack + 1 - A_windowfirst + SEQSPACE) % SEQSPACE;
    if (A_windowcount != 0 && !acked[ack] && pos <= WINDOWSIZE)
    {
      acked[ack] = true;

      /* packet is a new ACK*/
      if (TRACE > 0)
        printf("----A: ACK %d is not a duplicate\n", packet.acknum);
      new_ACKs++;

      /* slide window*/
      while (acked[A_windowfirst] && A_windowcount > 0)
      {
        A_windowfirst = (A_windowfirst + 1) % SEQSPACE;
        A_windowcount--;
      }

      if (A_windowcount == 0)
      {
        stoptimer(A);
        time_flag = false;
      }
    }
    else if (TRACE > 0)
      printf("----A: duplicate ACK received, do nothing!\n");
  }
  else if (TRACE > 0)
    printf("----A: corrupted ACK is received, do nothing!\n");
}

/* called when A's timer goes off */
void A_timerinterrupt(void)
{
  int i, pos;
  int flag;

  flag = false;

  for (i = 0; i < A_windowcount; i++)
  {
    pos = (A_windowfirst + i) % SEQSPACE;
    if (!acked[pos])
    {
      sendtime[pos] -= TICK;
      if (sendtime[pos] <= 0)
      {
        if (flag == false)
        {
          if (TRACE > 0)
            printf("----A: time out, resend packets!\n");
          flag = true;
        }
        if (TRACE > 0)
          printf("---A: resending packet %d\n", (sendbuffer[pos]).seqnum);
        tolayer3(A, sendbuffer[pos]);
        packets_resent++;
        sendtime[pos] = RTT;
      }
    }
  }
  if (A_windowcount > 0)
  {
    starttimer(A, TICK);
  }
  else
  {
    time_flag = false;
  }
}

/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init(void)
{
  int i;
  /* initialise A's window, buffer and sequence number */
  A_nextseqnum = 0; /* A starts with seq num 0, do not change this */
  A_windowfirst = 0;
  /* windowlast is where the last packet sent is stored.
      new packets are placed in winlast + 1
      so initially this is set to -1
    */
  A_windowcount = 0;

  time_flag = false;
  for (i = 0; i < SEQSPACE; i++)
  {
    acked[i] = true;
    sendtime[i] = 0;
  }
}

/********* Receiver (B)  variables and procedures ************/

static struct pkt recvbuffer[SEQSPACE]; /* array for storing packets received*/
static int B_windowfirst;               /* array indexes of the first packet received ack*/
static bool received[SEQSPACE];         /* array for storing have received*/
static int B_nextseqnum;                /* the sequence number for the next packets sent by B*/

/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt packet)
{
  struct pkt sendpkt;
  int i;
  int seq = packet.seqnum;
  int pos = (seq - B_windowfirst + SEQSPACE) % SEQSPACE;

  /* if not corrupted and received packet is in order */
  if ((!IsCorrupted(packet)) && pos < WINDOWSIZE)
  {
    if (TRACE > 0)
      printf("----B: packet %d is correctly received, send ACK!\n", packet.seqnum);
    packets_received++;

    /* send an ACK for the received packet */
    sendpkt.acknum = seq;
    if (!received[seq])
    {
      recvbuffer[seq] = packet;
      received[seq] = true;
    }
    while (received[B_windowfirst])
    {
      tolayer5(B, recvbuffer[B_windowfirst].payload);
      received[B_windowfirst] = false;
      B_windowfirst = (B_windowfirst + 1) % SEQSPACE;
    }
    sendpkt.seqnum = B_nextseqnum;
    B_nextseqnum = (B_nextseqnum + 1) % 2;

    /* we don't have any data to send. fill payload with 0's */
    for (i = 0; i < 20; i++)
      sendpkt.payload[i] = '0';

    /* computer checksum*/
    sendpkt.checksum = ComputeChecksum(sendpkt);

    /* send out packet*/
    tolayer3(B, sendpkt);
  }
  else if ((!IsCorrupted(packet)) && (B_windowfirst - seq + SEQSPACE) % SEQSPACE > 0 && (B_windowfirst - seq + SEQSPACE) % SEQSPACE <= WINDOWSIZE)
  {
    /* packet is corrupted or out of order resend last ACK */
    if (TRACE > 0)
      printf("----B: packet corrupted or not expected sequence number, resend ACK!\n");
    sendpkt.acknum = seq;

    /* create packet*/
    sendpkt.seqnum = B_nextseqnum;
    B_nextseqnum = (B_nextseqnum + 1) % 2;

    /* we don't have any data to send.  fill payload with 0's */
    for (i = 0; i < 20; i++)
      sendpkt.payload[i] = '0';

    /* computer checksum */
    sendpkt.checksum = ComputeChecksum(sendpkt);

    /* send out packet */
    tolayer3(B, sendpkt);
  }
  else
  {
  }
}

/* the following routine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init(void)
{
  int i;
  B_windowfirst = 0;
  B_nextseqnum = 1;
  for (i = 0; i < SEQSPACE; i++)
  {
    received[i] = false;
  }
}

/******************************************************************************
 * The following functions need be completed only for bi-directional messages *
 *****************************************************************************/

/* Note that with simplex transfer from a-to-B, there is no B_output() */
void B_output(struct msg message)
{
}

/* called when B's timer goes off */
void B_timerinterrupt(void)
{
}
