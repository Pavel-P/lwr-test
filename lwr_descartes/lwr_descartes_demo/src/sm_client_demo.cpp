/*
  Simple message client tester.

  Principles of operation: the foreground process reads simple message
  requests for joint moves and sends them to the server. Two
  background threads handle replies and joint states. The replies are
  printed when them come in, normally just after the foreground
  process sends the request. The states are read in and a global data
  structure is updated. This is printed out when the foreground
  process gets a blank line. The global state is mutexed so that a
  request to print out the state doesn't interfere with the state
  updating thread.

  The background process reply printing has the potential to
  interleave its output with the terminal, but normally the user would
  type in the request and then wait a bit before the next request, and
  the reply printing would finish before that. Interference would be
  an annoyance but would not be an error.
 */

#include <stddef.h>		// NULL, sizeof
#include <stdio.h>		// printf
#include <stdlib.h>		// atoi, strtof
#include <ctype.h>		// isspace

#include <ulapi.h>		// getopt, ulapi_socket_XXX

#include "simple_message_defs.h"

static bool debug = true;
enum {
  DEBUG_NONE = 0x00,
  DEBUG_REPLY = 0x01,
  DEBUG_STATE = 0x02,
  DEBUG_FEEDBACK = 0x04,
  DEBUG_RUN = 0x08,
  DEBUG_ALL = 0xFF
};
static int debug_level = DEBUG_ALL;

// Reply message client code ----------------------

struct reply_client_thread_args {
  void *thread;
  int id;
};

static void reply_client_thread_code(reply_client_thread_args *args)
{
  void *thread;
  int id;
  enum {INBUF_LEN = 1024};
  char inbuf[INBUF_LEN];
  char *ptr;
  int nchars;
  int nleft;
  int length;
  int message_type;
  cart_traj_pt_reply_message ctp_rep;

  thread = args->thread;
  id = args->id;

  while (true) {
    nchars = ulapi_socket_read(id, inbuf, sizeof(inbuf));
    if (nchars <= 0) break;
    inbuf[sizeof(inbuf)-1] = 0;
    ptr = inbuf;
    nleft = nchars;

    while (nleft > 0) {
      // get the length and type of the message
      memcpy(&length, ptr, sizeof(length));
      memcpy(&message_type, ptr + sizeof(length), sizeof(message_type));

      // switch on the message type and handle it
      switch (message_type) {
      case MESSAGE_PING:
	break;
      case MESSAGE_CART_TRAJ_PT:
    ctp_rep.read_cart_traj_pt_reply(ptr);
    if (debug && (debug_level & DEBUG_REPLY)) ctp_rep.print_cart_traj_pt_reply();
	break;
      default:
    printf("unknown reply type: %d\n", message_type);
    break;
      } // switch (message type)
      nleft -= (sizeof(length) + length);
      ptr += (sizeof(length) + length);
    } // while (nleft)
  }   // while (true)

  ulapi_socket_close(id);

  if (debug && (debug_level & DEBUG_RUN)) printf("joint state client handler %d done\n", id);

  return;
}


/* This is a simplified simple message client for communicating with descartes */
/* This client only supports cartesian trajectory points in the form:
   x y z r p y t
*/

int main(int argc, char *argv[])
{
  enum {HOST_LEN = 256};
  char host[HOST_LEN] = "localhost";
  int option;
  int ival;
  int message_port = MESSAGE_PORT_DEFAULT;
  int state_port = STATE_PORT_DEFAULT;
  int message_client_id;
  int state_client_id;
  ulapi_task_struct reply_client_thread;
  reply_client_thread_args rc_args; 
  cart_traj_pt_request_message ctp_req;
  ping_request_message p_req;

  // connect to message server
  if (message_port > 0) {
    message_client_id = ulapi_socket_get_client_id(message_port, host);
    if (message_client_id < 0) {
      fprintf(stderr, "can't connect to %s on port %d\n", host, message_port);
      return 1;
    }

    // start message reply handler
    ulapi_task_init(&reply_client_thread);
    rc_args.thread = reinterpret_cast<void *>(reply_client_thread);
    rc_args.id = message_client_id;
    ulapi_task_start(&reply_client_thread, reinterpret_cast<ulapi_task_code>(reply_client_thread_code), reinterpret_cast<void *>(&rc_args), ulapi_prio_highest(), 0);
  }

  printf("You are connected!\n");
  printf("q : quit\nb : start/stop batching\nTo send a cartesian trajectory point type 7 numbers: x y z r p y t\n");
// we in the foreground read input and update the robot model as needed
  bool done = false;
  while (! done)
  {
    enum {INBUF_LEN = 1024};
    char inbuf[INBUF_LEN];
    char *ptr;
    char *endptr;
    float f1, f2, f3, f4, f5, f6, f7;
    int nchars;
    float tmp_b;

    // print prompt
    printf("> ");
    fflush(stdout);
    // get input line
    if (NULL == fgets(inbuf, sizeof(inbuf), stdin)) break;
    // skip leading whitespace
    ptr = inbuf;
    puts(ptr);
    while (isspace(*ptr)) ptr++;
    // strip off trailing whitespace
    do
    {
      if (0 == *ptr)
      {		// blank line
          ctp_req.print_cart_traj_pt_request();
          break;
      }
      if ('q' == *ptr) //quit
      {
          done = true;
          break;
      }
      if ('b' == *ptr) //batching
      {   
          ulapi_socket_write(message_client_id, reinterpret_cast<char *>(&p_req), sizeof(p_req) );
          break;
      }
      if (7 != sscanf(ptr, "%f %f %f %f %f %f %f", 
                  &f1, &f2, &f3, &f4, &f5, &f6, &f7)) 
                  {
                      puts("malformed cartesian trajectory point");
                      break;
                  }
      else
      {
                      ctp_req.set_pos(f1, f2, f3, f4, f5, f6, f7);
                      ctp_req.set_seq_number(ctp_req.get_seq_number()+1);
                      nchars = ulapi_socket_write(message_client_id, reinterpret_cast<char *>(&ctp_req), sizeof(ctp_req));
                      if (nchars <= 0) break;
      }
    } while (false);		// do ... wrapper
  } // while (true)

  return 0;
}

