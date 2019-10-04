#if !defined(PLATFORM_APERIOS) && !defined(__APPLE__)

// Mary client code for Tekkotsu, modified from the Mary files
// examples/client/c++/mary_client.cc and demo_mary_client.cc

//**************** Original DFKI GmbH copyright notice:
/**
 * Copyright 2000-2006 DFKI GmbH.
 * All Rights Reserved.  Use is subject to license terms.
 * 
 * Permission is hereby granted, free of charge, to use and distribute
 * this software and its documentation without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of this work, and to
 * permit persons to whom this work is furnished to do so, subject to
 * the following conditions:
 * 
 * 1. The code must retain the above copyright notice, this list of
 *    conditions and the following disclaimer.
 * 2. Any modifications must be clearly marked as such.
 * 3. Original authors' names are not deleted.
 * 4. The authors' names are not used to endorse or promote products
 *    derived from this software without specific prior written
 *    permission.
 *
 * DFKI GMBH AND THE CONTRIBUTORS TO THIS WORK DISCLAIM ALL WARRANTIES WITH
 * REGARD TO THIS SOFTWARE, INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS, IN NO EVENT SHALL DFKI GMBH NOR THE
 * CONTRIBUTORS BE LIABLE FOR ANY SPECIAL, INDIRECT OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
 * PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS
 * ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF
 * THIS SOFTWARE.
 */

#include <netinet/in.h>
#include <sys/socket.h>
#include <netdb.h>
#include <cstdlib>
#include <cstring>
#include <string>
#include <iostream>
#include <unistd.h>  // for fork, execlp, _exit, and close

#include "MaryClient.h"

void launchMaryServer() {
  pid_t child_id = fork();
  if ( child_id == 0 ) {
    char* tekrootval = getenv("TEKKOTSU_ROOT");
    std::string const tekkotsuRoot = tekrootval==NULL ? "/usr/local/Tekkotsu" : std::string(tekrootval);
    std::string const maryServerName = "tekk-maryserver";
    std::string const maryServerPath = tekkotsuRoot + "/tools/Mary/bin/" + maryServerName;
    execlp(maryServerPath.c_str(), maryServerName.c_str(), NULL);
    // If we get here, the execlp() failed
    std::cerr << "ERROR: failed to launch Mary server from " << maryServerPath << std::endl
	      << "Check that TEKKOTSU_ROOT is set properly." << std::endl;
    _exit(0);
  }
}

int maryQuery(std::string& result,
	      const std::string& inputText,
	      const std::string& voice) {

  int server_port = 59125;
  int status;

  struct hostent* hostInfo = gethostbyname("localhost");
  if (hostInfo == NULL)
    return -2;

  // Create a tcp connection to the Mary server's info socket 
  int maryInfoSocket = socket(AF_INET, SOCK_STREAM, 0);
  if (maryInfoSocket == -1)
    return -2;

  // autoflush stdout, bind and connect
  struct sockaddr_in maryClient;
  maryClient.sin_family = AF_INET;
  maryClient.sin_port = htons(0);
  maryClient.sin_addr.s_addr = INADDR_ANY;
  status = bind(maryInfoSocket, (struct sockaddr*) &maryClient, sizeof(maryClient));
  if (status != 0)
    return -2;

  struct sockaddr_in maryServer;
  maryServer.sin_family = AF_INET;
  maryServer.sin_port = htons(server_port);
  memcpy((char*) &maryServer.sin_addr.s_addr, hostInfo->h_addr_list [0], hostInfo->h_length);
  status = connect(maryInfoSocket, (struct sockaddr*) &maryServer, sizeof(maryServer));
  if (status != 0)
    return -2;

  // prepare the request
  std::string const query = "MARY IN=TEXT_EN OUT=AUDIO AUDIO=WAVE VOICE=" + voice + "\012\015";
  if (send(maryInfoSocket, query.c_str(), query.size(), 0) == -1)
    return -2;

  // receive the request id
  char id [32] = "";
  if (recv(maryInfoSocket, id, 32, 0) == -1)
    return -2;

  // Create a tcp connection to the Mary server's data socket
  int maryDataSocket = socket(AF_INET, SOCK_STREAM, 0);
  if (maryDataSocket == -1)
    return -2;

  // autoflush stdout, bind and connect
  maryClient.sin_family = AF_INET;
  maryClient.sin_port = htons(0);
  maryClient.sin_addr.s_addr = INADDR_ANY;

  status = bind(maryDataSocket, (struct sockaddr*) &maryClient, sizeof(maryClient));
  if (status != 0)
    return -2;

  maryServer.sin_family = AF_INET;
  maryServer.sin_port = htons(server_port);
  memcpy((char*) &maryServer.sin_addr.s_addr, hostInfo->h_addr_list [0], hostInfo->h_length);
  status = connect(maryDataSocket, (struct sockaddr*) &maryServer, sizeof(maryServer));
  if (status != 0)
    return -2;

  // send the request id to the Mary server
  if (send(maryDataSocket, id, strlen(id), 0) == -1)
    return -2;

  // send the query to the Mary server
  if (send(maryDataSocket, inputText.c_str(), inputText.size(), 0) == -1)
    return -2;
  if (send(maryDataSocket, "\012\015", 2, 0) == -1)
    return -2;
  
  shutdown(maryDataSocket, 1);

  unsigned int total_bytes = 0;
  size_t recv_bytes = 0;
  char data [1024] = "";
  result[0] = '\0';

  // receive the request result
  do {
    data [0] = '\0';
    recv_bytes = recv(maryDataSocket, data, 1024, 0);
    if (recv_bytes == -1U)
      return -2;
    else if (recv_bytes > 0) {
      //std::cerr << "("<<recv_bytes<<")";
      total_bytes += recv_bytes;
      data [recv_bytes] = '\0';

      for (size_t i=0; i<recv_bytes; i++)
	result += data [i];
    }
  } while (recv_bytes != 0);

  if (result.size() != total_bytes) {
    std::cerr << "error: total bytes received != result bytes!" << std::endl;
    std::cerr << "       total bytes received = " << total_bytes << std::endl;
    std::cerr << "       result bytes = " << result.size() << std::endl;
  }

  // receive the request error
  do {
    data [0] = '\0';
    recv_bytes = recv(maryInfoSocket, data, 1024, 0);
    if (recv_bytes == -1U)
      return -2;
    else if (recv_bytes > 0) {
      std::cerr << std::endl << "Mary error code: " << data << std::endl;
      return -3;
    }
  } while (recv_bytes != 0);

  close(maryInfoSocket);
  close(maryDataSocket);
  return 0;
}

#endif
