#ifndef RVDRAW_H
#define RVDRAW_H

/*
 *  Copyright (C) 2011 Justin Stoecker
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */


#include <cstdio>
#include <cstring>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <string>
#include <math.h>
#include <map>
#include <boost/lexical_cast.hpp>
#include <sstream>
#include <iomanip>

#define ROBOVIS_PORT "32769"

using namespace std;

inline int writeCharToBuf(unsigned char* buf, unsigned char value) {
    *buf = value;
    return 1;
}

inline int writeFloatToBuf(unsigned char* buf, float value) {
    char temp[20];
    sprintf(temp, "%6f", value);
    memcpy(buf, temp, 6);
    return 6;
}

inline int writeColorToBuf(unsigned char* buf, const float* color, int channels) {
    int i;
    for (i = 0; i < channels; i++)
        writeCharToBuf(buf+i, (unsigned char)(color[i]*255));
    return i;
}

inline int writeStringToBuf(unsigned char* buf, const string* text) {
    long i = 0;
    if (text != NULL)
        i += text->copy((char*)buf+i, text->length(), 0);
    i += writeCharToBuf(buf+i, 0);
    return i;
}

//for the drawings map
struct DrawObject {
    string id;
    string buf;
};

class RVSender
{
private:

    static int side; //either SIDE_RIGHT or SIDE_LEFT
    static int uNum;
    static long uniqueIdNum;
    map<string, string> drawings;
    map<int, string> sideMap;
    char getTeamAgent(int uNum, int side); //for AgentAnnotation commands
    void getColor(int uNum, int side, float &r, float &g, float &b);
    string getMyId();
    string getDrawingId(const string* name);
    string getUniqueId(long unique);
    string getUniqueId();
    void flipPolygon(float *v, const int numVerts); //for agents on SIDE_RIGHT
    void updateDrawings(string id, string buf);

    int sockfd;
    struct addrinfo p;
    bool socketCreated;

    unsigned char* newBufferSwap(const string* name, int* bufSize);
    unsigned char* newCircle(const float* center,
                             float radius, float thickness,
                             const float* color,
                             const string* setName,
                             int *bufSize);
    unsigned char* newLine(const float* a,
                           const float* b,
                           float thickness,
                           const float* color,
                           const string* setName,
                           int* bufSize);
    unsigned char* newPoint(const float* p,
                            float size,
                            const float* color,
                            const string* setName,
                            int* bufSize);
    unsigned char* newSphere(const float* p,
                             float radius,
                             const float* color,
                             const string* setName,
                             int* bufSize);
    unsigned char* newPolygon(const float* v,
                              int numVerts,
                              const float* color,
                              const string* setName,
                              int* bufSize);
    unsigned char* newAnnotation(const string *txt,
                                 const float *p,
                                 const float* color,
                                 const string* setName,
                                 int* bufSize);
    unsigned char* newAgentAnnotation(const string *txt,
                                      const char teamAgent,
                                      const float* color,
                                      int* bufSize);
    unsigned char* newRemoveAgentAnnotation(const char teamAgent,
                                            int* bufSize);
    unsigned char* newSelectAgent(const char teamAgent,
                                  int* bufSize);

public:
    RVSender();
    RVSender(int sockfd_, struct addrinfo p);
    ~RVSender();

    inline int getSockFD() {
        return sockfd;
    }
    inline struct addrinfo getP() {
        return p;
    }

    inline void setUNum(const int &uNum) {
        this->uNum = uNum;
    }
    inline void setSide(const int &side) {
        this->side = side;
    }

    inline bool isInit() {
        return side != -1 && uNum !=1 -1;
    }

    //this must match the switch in getColor. See rvdraw.cc before changing
    enum Color {
        RED       = 1, ORANGE      =  2, YELLOW    =  3, GREEN     = 4,
        BLUEGREEN = 5, LIGHTBLUE   =  6, BLUE      =  7, DARKBLUE  = 8,
        VIOLET    = 9, PINK        = 10, MAGENTA   = 11
    };


    /*
     *  These draw commands support animation.
     *
     *  Every time a draw command is called, a map is updated with
     *  key = string name (the first argument to each of these commands), and
     *  value = the draw command's buffer (no need to understand that part).
     *
     *  Call refresh() to write these buffers to the screen. I recommend putting refresh() in
     *  NaoBehavior::Think().
     *
     *  Call clear() to erase what's in the buffers. That allows old shapes to be removed from the
     *  screen and prevents clutter. It allows you to know what your agents are thinking right now.
     *  I recommend putting clear() in NaoBehavior::selectSkill().
     *
     *  It's important to give shapes unique a unique 'string name' argument.
     *  A shape created with some 'string name' argument will overwrite an older shape with the
     *  same 'string name' argument. That's how the animation works.
     *
     *  You can use the RVSender::Color enum to specify colors, or specify your own r,g,b values.
     *  If you don't specify the color, it will default to a color specific to the player's
     *  uNum (very useful).
     */

    /* draws all elements of drawings map to the screen */
    void refresh();

    /* erases all elements of drawings map */
    void clear();

    void drawCircle(string name, double x, double y, double radius,
                    RVSender::Color c=(RVSender::Color)uNum);
    void drawCircle(string name, double x, double y, double radius, float r, float g, float b);

    void drawLine(string name, double x1, double y1, double x2, double y2,
                  RVSender::Color c=(RVSender::Color)uNum);
    void drawLine(string name, double x1, double y1, double x2, double y2, float r, float g, float b);

    void drawText(string name, string text, double x, double y,
                  RVSender::Color c=(RVSender::Color)uNum);
    void drawText(string name, string text, double x, double y, float r, float g, float b);

    void drawPoint(string name, double x, double y, double radius,
                   RVSender::Color c=(RVSender::Color)uNum);
    void drawPoint(string name, double x, double y, double radius, float r, float g, float b);

    void drawSphere(string name, double x, double y, double z, double radius,
                    RVSender::Color c=(RVSender::Color)uNum);
    void drawSphere(string name, double x, double y, double z, double radius,
                    float r, float g, float b);

    /* format your *v array as {x1, y1, z1, x2, y2, z2, ...} */
    /* the 'a' arguments are for alpha channel (transparency) */
    void drawPolygon(string name, float *v, int numVerts);
    void drawPolygon(string name, float *v, int numVerts, float a);
    void drawPolygon(string name, float *v, int numVerts, RVSender::Color c=(RVSender::Color)uNum,
                     float a=1.0f);
    void drawPolygon(string name, float *v, int numVerts, float r, float g, float b, float a=1.0f);

    /*
     *  These draw commands are for static shapes that remain on the screen indefinitely,
     *  or until you call clearStaticDrawings() to remove all of them. Use that method on
     *  a timer to keep only a recent history of drawings on screen.
     */

    /* use this to erase all static shapes. */
    void clearStaticDrawings();

    void drawCircle(double x, double y, double radius,
                    RVSender::Color c=(RVSender::Color)uNum);
    void drawCircle(double x, double y, double radius, float r, float g, float b);

    void drawLine(double x1, double y1, double x2, double y2,
                  RVSender::Color c=(RVSender::Color)uNum);
    void drawLine(double x1, double y1, double x2, double y2, float r, float g, float b);

    //these two are causing strange errors in roboviz
    /*void drawText(string text, double x, double y,
                    RVSender::Color c=(RVSender::Color)uNum);
    void drawText(string text, double x, double y, float r, float g, float b);*/

    void drawPoint(double x, double y, double radius,
                   RVSender::Color c=(RVSender::Color)uNum);
    void drawPoint(double x, double y, double radius, float r, float g, float b);

    void drawSphere(double x, double y, double z, double radius,
                    RVSender::Color c=(RVSender::Color)uNum);
    void drawSphere(double x, double y, double z, double radius,
                    float r, float g, float b);

    void drawPolygon(float *v, int numVerts);
    void drawPolygon(float *v, int numVerts, float a);
    void drawPolygon(float *v, int numVerts, RVSender::Color c, float a=1.0f);
    void drawPolygon(float *v, int numVerts, float r, float g, float b, float a=1.0f);

    /*
     *  These agentText commands work differently. No animation involved, you don't need
     *  the refresh() and clear() commands for these to work. No unique 'string name' either.
     */

    void drawText(string text, double x, double y,
                  RVSender::Color c=(RVSender::Color)uNum);
    void drawText(string text, double x, double y, float r, float g, float b);

    /* draw to self */
    void drawAgentText(string text, RVSender::Color c=(RVSender::Color)uNum);
    void drawAgentText(string text, float r, float g, float b);

    /* draw to teammate with uniform "uNum" */
    void drawAgentText(string text, int uNum, RVSender::Color c=(RVSender::Color)uNum);
    void drawAgentText(string text, int uNum, float r, float g, float b);

    /* draw to player on team "side", with uniform "uNum" */
    void drawAgentText(string text, int uNum, int side, RVSender::Color c=(RVSender::Color)uNum);
    void drawAgentText(string text, int uNum, int side, float r, float g, float b);

    /* defaults to removing text from self, but may specify a teammate, or any player on the field */
    void removeAgentText(int u=uNum, int s=side);

    void selectAgent(int u=uNum, int s=side);


    /* old draw commands, don't support the animation */
    void swapBuffers(const string* setName);
    void drawLine(float x1, float y1, float z1,
                  float x2, float y2, float z2,
                  float thickness,
                  float r, float g, float b,
                  const string* setName);
    void drawCircle(float x, float y, float radius,
                    float thickness,
                    float r, float g, float b,
                    const string* setName);
    void drawSphere(float x, float y, float z,
                    float radius,
                    float r, float g, float b,
                    const string* setName);
    void drawPoint(float x, float y, float z,
                   float size,
                   float r, float g, float b,
                   const string *setName);
    void drawPolygon(const float *v, int numVerts,
                     float r, float g, float b,
                     float a,
                     const string *setName);
    void drawAnnotation(const string *txt,
                        float x, float y, float z,
                        float r, float g, float b,
                        const string *setName);
    void drawAgentAnnotation(const string *txt,
                             char teamAgent,
                             float r, float g, float b);
    void removeAgentAnnotation(char teamAgent);
    void selectAgent(char teamAgent);
};

#endif // !RVDRAW_H
