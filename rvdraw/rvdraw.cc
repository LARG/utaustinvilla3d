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

#include "rvdraw.h"
#include "../worldmodel/worldmodel.h" //for SIDE_RIGHT and SIDE_LEFT macros

extern string mHost;

int RVSender::side;
int RVSender::uNum;
long RVSender::uniqueIdNum;


/**
 * sockfd = -1 on error!
 * This creates a socket, which
 * is then closed by the
 * destructor. Do not close the
 * socket!
 */
RVSender::RVSender()
{
    socketCreated = false;
    sockfd = -1;

    struct addrinfo hints, *servinfo, *p_ptr;
    int rv;
    int numbytes;

    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;

    if ((rv = getaddrinfo(mHost.c_str(), ROBOVIS_PORT, &hints, &servinfo)) != 0) {
        fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
        return;
    }

    // loop through all the results and make a socket
    for(p_ptr = servinfo; p_ptr != NULL; p_ptr = p_ptr->ai_next) {
        if ((sockfd = socket(p_ptr->ai_family, p_ptr->ai_socktype,
                             p_ptr->ai_protocol)) == -1) {
            perror("socket");
            continue;
        }

        p = *p_ptr;

        break;
    }

    if (p_ptr == NULL) {
        fprintf(stderr, "failed to bind socket\n");
        freeaddrinfo(servinfo);
        return;
    }

    socketCreated = true;

    //freeaddrinfo(servinfo);
    side = -1;
    uNum = -1;
    sideMap[SIDE_LEFT]  = "L";
    sideMap[SIDE_RIGHT] = "R";
}

/**
 * Give a socket and addrinfo.
 * Whoever created the socket is
 * responsible for closing it!
 */
RVSender::RVSender(int sockfd_, struct addrinfo p_)
{
    socketCreated = false;
    p = p_;
    sockfd = sockfd_;
}

RVSender::~RVSender()
{
    if (socketCreated)
        close(sockfd);
}

/*
int RVSender::getSockFD()
 {
   return sockfd;
 }

struct addrinfo RVSender::getP()
 {
   return p;
 }
*/


char RVSender::getTeamAgent(int uNum, int side) {
    return (side == SIDE_LEFT ? uNum : uNum + 128) - 1;
}

//drawing colors are uniform & team specific
//this must match the enum in rvdraw.h
void RVSender::getColor(int uNum, int side, float &r, float &g, float &b) {
    switch(uNum) {
    case 1: //red
        r = 1;
        g = 0;
        b = 0;
        break;
    case 2: //orange
        r = 1;
        g = 0.5f;
        b = 0;
        break;
    case 3: //yellow
        r = 1;
        g = 1;
        b = 0;
        break;
    case 4: //green
        r = 0;
        g = 1;
        b = 0;
        break;
    case 5: //blue-green
        r = 0;
        g = 1;
        b = 0.5f;
        break;
    case 6: //light blue
        r = 0;
        g = 1;
        b = 1;
        break;
    case 7: //blue
        r = 0;
        g = 0.5f;
        b = 1;
        break;
    case 8: //dark blue
        r = 0;
        g = 0;
        b = 1;
        break;
    case 9: //violet
        r = 0.5f;
        g = 0;
        b = 1;
        break;
    case 10: //pink
        r = 1;
        g = 0;
        b = 1;
        break;
    case 11: //magenta
        r = 1;
        g = 0;
        b = 0.5f;
        break;
    default:
        r = 0;
        g = 0;
        b = 0;
    }
    // players on the right team get darker colors
    if (side == SIDE_RIGHT) {
        r /= 2.0f;
        g /= 2.0f;
        b /= 2.0f;
    }
}

string RVSender::getMyId() {
    stringstream stream;
    stream << sideMap[side] << '.' << std::setfill('0') <<
           std::setw(2) << boost::lexical_cast<std::string>(uNum);
    return stream.str();
}

string RVSender::getDrawingId(const string* name) {
    stringstream stream;
    stream << getMyId() << '.' << *name;
    return stream.str();
}

string RVSender::getUniqueId(long unique) {
    stringstream stream;
    stream << '_' << unique << '.' << getMyId();
    return stream.str();
}

string RVSender::getUniqueId() {
    return getUniqueId(uniqueIdNum++);
}

void RVSender::flipPolygon(float *v, int numVerts) {
    /* must flip each number, there are three numbers (x, y, z) for each vertex */
    for (int i = 0; i < numVerts * 3; i++) {
        v[i] = -v[i];
    }
}

void RVSender::updateDrawings(string id, string buf) {
    drawings[id] = buf;
}


// === Public Methods ===
// Protect them with a check for isInit()

void RVSender::clear() {
    if (!isInit()) {
        return;
    }
    for (map<string,string>::iterator it = drawings.begin();
            it != drawings.end(); ++it) {
        it->second = "";
    }
}

void RVSender::refresh() {
    if (!isInit()) {
        return;
    }
    string myId(getMyId());
    swapBuffers(&myId);
}

void RVSender::clearStaticDrawings() {
    if (!isInit()) {
        return;
    }
    for (long i = 0; i < uniqueIdNum; i++) {
        string uniqueId = getUniqueId(i);
        swapBuffers(&uniqueId);
    }
    uniqueIdNum = 0;
}

void RVSender::drawCircle(string name, double x, double y, double radius,
                          RVSender::Color c) {
    if (!isInit()) {
        return;
    }
    float r, g, b;
    getColor(c, side, r, g, b);
    drawCircle(name, x, y, radius, r, g, b);
}

void RVSender::drawCircle(string name, double x, double y, double radius, float r, float g, float b) {
    if (!isInit()) {
        return;
    }
    if (side == SIDE_RIGHT) {
        x *= -1;
        y *= -1;
    }
    string id = getDrawingId(&name);
    string buf = string(id);
    drawCircle(x, y, radius, 3, r, g, b, &buf);
    updateDrawings(id, buf);
}

void RVSender::drawCircle(double x, double y, double radius, RVSender::Color c) {
    if (!isInit()) {
        return;
    }
    float r, g, b;
    getColor(c, side, r, g, b);
    drawCircle(x, y, radius, r, g, b);
}

void RVSender::drawCircle(double x, double y, double radius, float r, float g, float b) {
    if (!isInit()) {
        return;
    }
    if (side == SIDE_RIGHT) {
        x *= -1;
        y *= -1;
    }
    string id = getUniqueId();
    string buf = string(id);
    drawCircle(x, y, radius, 3, r, g, b, &buf);
    swapBuffers(&id);
}

void RVSender::drawLine(string name, double x1, double y1, double x2, double y2,
                        RVSender::Color c) {
    if (!isInit()) {
        return;
    }
    float r, g, b;
    getColor(c, side, r, g, b);
    drawLine(name, x1, y1, x2, y2, r, g, b);
}

void RVSender::drawLine(string name, double x1, double y1, double x2, double y2,
                        float r, float g, float b) {
    if (!isInit()) {
        return;
    }
    if (side == SIDE_RIGHT) {
        x1 *= -1;
        y1 *= -1;
        x2 *= -1;
        y2 *= -1;
    }
    string id = getDrawingId(&name);
    string buf = string(id);
    drawLine(x1, y1, 0, x2, y2, 0, 3, r, g, b, &buf);
    updateDrawings(id, buf);
}

void RVSender::drawLine(double x1, double y1, double x2, double y2, RVSender::Color c) {
    if (!isInit()) {
        return;
    }
    float r, g, b;
    getColor(c, side, r, g, b);
    drawLine(x1, y1, x2, y2, r, g, b);
}

void RVSender::drawLine(double x1, double y1, double x2, double y2, float r, float g, float b) {
    if (!isInit()) {
        return;
    }
    if (side == SIDE_RIGHT) {
        x1 *= -1;
        y1 *= -1;
        x2 *= -1;
        y2 *= -1;
    }
    string id = getUniqueId();
    string buf = string(id);
    drawLine(x1, y1, 0, x2, y2, 0, 3, r, g, b, &buf);
    swapBuffers(&id);
}

void RVSender::drawText(string name, string text, double x, double y,
                        RVSender::Color c) {
    if (!isInit()) {
        return;
    }
    float r, g, b;
    getColor(c, side, r, g, b);
    drawText(name, text, x, y, r, g, b);
}

void RVSender::drawText(string name, string text, double x, double y,
                        float r, float g, float b) {
    if (!isInit()) {
        return;
    }
    if (side == SIDE_RIGHT) {
        x *= -1;
        y *= -1;
    }
    string id = getDrawingId(&name);
    string buf = string(id);
    drawAnnotation(&text, x, y, 0, r, g, b, &buf);
    updateDrawings(id, buf);
}

void RVSender::drawText(string text, double x, double y,
                        RVSender::Color c) {
    if (!isInit()) {
        return;
    }
    float r, g, b;
    getColor(c, side, r, g, b);
    drawText(text, x, y, r, g, b);
}

void RVSender::drawText(string text, double x, double y,
                        float r, float g, float b) {
    if (!isInit()) {
        return;
    }
    if (side == SIDE_RIGHT) {
        x *= -1;
        y *= -1;
    }
    string id = getUniqueId();
    string buf = string(id);
    drawAnnotation(&text, x, y, 0, r, g, b, &buf);
    swapBuffers(&id);
}

void RVSender::drawPoint(string name, double x, double y, double radius,
                         RVSender::Color c) {
    if (!isInit()) {
        return;
    }
    float r, g, b;
    getColor(c, side, r, g, b);
    drawPoint(name, x, y, radius, r, g, b);
}

void RVSender::drawPoint(string name, double x, double y, double radius,
                         float r, float g, float b) {
    if (!isInit()) {
        return;
    }
    if (side == SIDE_RIGHT) {
        x *= -1;
        y *= -1;
    }
    string id = getDrawingId(&name);
    string buf = string(id);
    drawPoint(x, y, 0, radius, r, g, b, &buf);
    updateDrawings(id, buf);
}

void RVSender::drawPoint(double x, double y, double radius,
                         RVSender::Color c) {
    if (!isInit()) {
        return;
    }
    float r, g, b;
    getColor(c, side, r, g, b);
    drawPoint(x, y, radius, r, g, b);
}

void RVSender::drawPoint(double x, double y, double radius,
                         float r, float g, float b) {
    if (!isInit()) {
        return;
    }
    if (side == SIDE_RIGHT) {
        x *= -1;
        y *= -1;
    }
    string id = getUniqueId();
    string buf = string(id);
    drawPoint(x, y, 0, radius, r, g, b, &buf);
    swapBuffers(&id);
}

void RVSender::drawSphere(string name, double x, double y, double z,
                          double radius, RVSender::Color c) {
    if (!isInit()) {
        return;
    }
    float r, g, b;
    getColor(c, side, r, g, b);
    drawSphere(name, x, y, z, radius, r, g, b);
}

void RVSender::drawSphere(string name, double x, double y, double z,
                          double radius, float r, float g, float b) {
    if (!isInit()) {
        return;
    }
    if (side == SIDE_RIGHT) {
        x *= -1;
        y *= -1;
    }
    string id = getDrawingId(&name);
    string buf = string(id);
    drawSphere(x, y, 0, radius, r, g, b, &buf);
    updateDrawings(id, buf);
}

void RVSender::drawSphere(double x, double y, double z,
                          double radius, RVSender::Color c) {
    if (!isInit()) {
        return;
    }
    float r, g, b;
    getColor(c, side, r, g, b);
    drawSphere(x, y, z, radius, r, g, b);
}

void RVSender::drawSphere(double x, double y, double z,
                          double radius, float r, float g, float b) {
    if (!isInit()) {
        return;
    }
    if (side == SIDE_RIGHT) {
        x *= -1;
        y *= -1;
    }
    string id = getUniqueId();
    string buf = string(id);
    drawSphere(x, y, 0, radius, r, g, b, &buf);
    swapBuffers(&id);
}

void RVSender::drawPolygon(string name, float *v, int numVerts, float a) {
    float r, g, b;
    if (!isInit()) {
        return;
    }
    getColor(uNum, side, r, g, b);
    drawPolygon(name, v, numVerts, r, g, b, a);
}

void RVSender::drawPolygon(string name, float *v, int numVerts,
                           RVSender::Color c, float a) {
    if (!isInit()) {
        return;
    }
    float r, g, b;
    getColor(c, side, r, g, b);
    drawPolygon(name, v, numVerts, r, g, b, a);
}

void RVSender::drawPolygon(string name, float *v, int numVerts,
                           float r, float g, float b, float a) {
    if (!isInit()) {
        return;
    }
    if (side == SIDE_RIGHT) {
        flipPolygon(v, numVerts);
    }
    string id = getDrawingId(&name);
    string buf = string(id);
    drawPolygon((float*)v, numVerts, r, g, b, a, &buf);
    updateDrawings(id, buf);
}

void RVSender::drawPolygon(float *v, int numVerts) {
    if (!isInit()) {
        return;
    }
    drawPolygon(v, numVerts, 1);
}

void RVSender::drawPolygon(float *v, int numVerts, float a) {
    if (!isInit()) {
        return;
    }
    float r, g, b;
    getColor(uNum, side, r, g, b);
    drawPolygon(v, numVerts, r, g, b, a);
}

void RVSender::drawPolygon(float *v, int numVerts,
                           RVSender::Color c, float a) {
    if (!isInit()) {
        return;
    }
    float r, g, b;
    getColor(c, side, r, g, b);
    drawPolygon(v, numVerts, r, g, b, a);
}

void RVSender::drawPolygon(float *v, int numVerts,
                           float r, float g, float b, float a) {
    if (!isInit()) {
        return;
    }
    if (side == SIDE_RIGHT) {
        flipPolygon(v, numVerts);
    }
    string id = getUniqueId();
    string buf = string(id);
    drawPolygon((float*)v, numVerts, r, g, b, a, &buf);
    swapBuffers(&id);
}

void RVSender::drawAgentText(string text, RVSender::Color c) {
    if (!isInit()) {
        return;
    }
    float r, g, b;
    getColor(c, side, r, g, b);
    drawAgentText(text, uNum, side, r, g, b);
}

void RVSender::drawAgentText(string text, float r, float g, float b) {
    if (!isInit()) {
        return;
    }
    drawAgentText(text, uNum, side, r, g, b);
}

void RVSender::drawAgentText(string text, int uNum, RVSender::Color c) {
    if (!isInit()) {
        return;
    }
    float r, g, b;
    getColor(c, side, r, g, b);
    drawAgentText(text, uNum, side, r, g, b);
}

void RVSender::drawAgentText(string text, int uNum, float r, float g, float b) {
    if (!isInit()) {
        return;
    }
    drawAgentText(text, uNum, side, r, g, b);
}

void RVSender::drawAgentText(string text, int uNum, int side, RVSender::Color c) {
    if (!isInit()) {
        return;
    }
    float r, g, b;
    getColor(c, side, r, g, b);
    drawAgentText(text, uNum, side, r, g, b);
}

void RVSender::drawAgentText(string text, int uNum, int side, float r, float g, float b) {
    if (!isInit()) {
        return;
    }
    char teamAgent = getTeamAgent(uNum, side);
    drawAgentAnnotation(&text, teamAgent, r, g, b);
}

void RVSender::removeAgentText(int u, int s) {
    if (!isInit()) {
        return;
    }
    char teamAgent = getTeamAgent(u, s);
    removeAgentAnnotation(teamAgent);
}

void RVSender::selectAgent(int u, int s) {
    if (!isInit()) {
        return;
    }
    char teamAgent = getTeamAgent(u, s);
    selectAgent(teamAgent);
}

void RVSender::swapBuffers(const string* setName) {
    if (!isInit()) {
        return;
    }
    int bufSize = -1;
    unsigned char* buf = newBufferSwap(setName, &bufSize);
    sendto(sockfd, buf, bufSize, 0, p.ai_addr, p.ai_addrlen);
    delete[] buf;
}

void RVSender::drawLine(float x1, float y1, float z1, float x2, float y2, float z2, float thickness, float r, float g, float b,
                        const string* setName) {
    if (!isInit()) {
        return;
    }
    float pa[3] = {x1,y1,z1};
    float pb[3] = {x2,y2,z2};
    float color[3] = {r,g,b};

    int bufSize = -1;
    unsigned char* buf = newLine(pa, pb, thickness, color, setName, &bufSize);
    sendto(sockfd, buf, bufSize, 0, p.ai_addr, p.ai_addrlen);
    delete[] buf;
}

void RVSender::drawCircle(float x, float y, float radius, float thickness, float r, float g, float b, const string* setName) {
    if (!isInit()) {
        return;
    }
    float center[2] = {x,y};
    float color[3] = {r,g,b};

    int bufSize = -1;
    unsigned char* buf = newCircle(center, radius, thickness, color, setName, &bufSize);
    sendto(sockfd, buf, bufSize, 0, p.ai_addr, p.ai_addrlen);
    delete[] buf;
}

void RVSender::drawSphere(float x, float y, float z, float radius, float r, float g, float b, const string* setName) {
    if (!isInit()) {
        return;
    }
    float center[3] = {x,y,z};
    float color[3] = {r,g,b};

    int bufSize = -1;
    unsigned char* buf = newSphere(center, radius, color, setName, &bufSize);
    sendto(sockfd, buf, bufSize, 0, p.ai_addr, p.ai_addrlen);
    delete[] buf;
}

void RVSender::drawPoint(float x, float y, float z, float size, float r, float g, float b, const string* setName) {
    if (!isInit()) {
        return;
    }
    float center[3] = {x,y,z};
    float color[3] = {r,g,b};

    //printf("Point: (%f, %f, %f)\n", x, y, z);

    int bufSize = -1;
    unsigned char* buf = newPoint(center, size, color, setName, &bufSize);
    sendto(sockfd, buf, bufSize, 0, p.ai_addr, p.ai_addrlen);
    delete[] buf;
}

void RVSender::drawPolygon(const float* v, int numVerts, float r, float g, float b, float a,
                           const string* setName) {
    if (!isInit()) {
        return;
    }
    float color[4] = {r,g,b,a};

    int bufSize = -1;
    unsigned char* buf = newPolygon(v, numVerts, color, setName, &bufSize);
    sendto(sockfd, buf, bufSize, 0, p.ai_addr, p.ai_addrlen);
    delete[] buf;
}

void RVSender::drawAnnotation(const string *txt, float x, float y, float z,
                              float r, float g, float b, const string *setName)
{
    if (!isInit()) {
        return;
    }
    float color[3] = {r,g,b};
    float point[3] = {x,y,z};
    int bufSize = -1;
    unsigned char *buf = newAnnotation(txt, point, color, setName, &bufSize);
    sendto(sockfd, buf, bufSize, 0, p.ai_addr, p.ai_addrlen);
    delete[] buf;
}

void RVSender::drawAgentAnnotation(const string *txt, char teamAgent,
                                   float r, float g, float b)
{
    if (!isInit()) {
        return;
    }
    float color[3] = {r,g,b};
    int bufSize = -1;
    unsigned char *buf = newAgentAnnotation(txt, teamAgent, color, &bufSize);
    sendto(sockfd, buf, bufSize, 0, p.ai_addr, p.ai_addrlen);
    delete[] buf;
}

void RVSender::removeAgentAnnotation(char teamAgent)
{
    if (!isInit()) {
        return;
    }
    int bufSize = -1;
    unsigned char *buf = newRemoveAgentAnnotation(teamAgent, &bufSize);
    sendto(sockfd, buf, bufSize, 0, p.ai_addr, p.ai_addrlen);
    delete[] buf;
}

void RVSender::selectAgent(char teamAgent)
{
    if (!isInit()) {
        return;
    }
    int bufSize = -1;
    unsigned char *buf = newSelectAgent(teamAgent, &bufSize);
    sendto(sockfd, buf, bufSize, 0, p.ai_addr, p.ai_addrlen);
    delete[] buf;
}

// The following commands allocate memory, if you use them be sure to deallocate
// the memory later.

unsigned char* RVSender::newBufferSwap(const string* name, int* bufSize) {
    *bufSize = 3 + ((name != NULL) ? name->length() : 0);
    unsigned char* buf = new unsigned char[*bufSize];

    long i = 0;
    i += writeCharToBuf(buf+i, 0);
    i += writeCharToBuf(buf+i, 0);
    i += writeStringToBuf(buf+i, name);

    return buf;
}

unsigned char* RVSender::newCircle(const float* center, float radius, float thickness,
                                   const float* color, const string* setName, int* bufSize) {

    *bufSize = 30 + ((setName != NULL) ? setName->length() : 0);
    unsigned char* buf = new unsigned char[*bufSize];

    long i = 0;
    i += writeCharToBuf(buf+i, 1);
    i += writeCharToBuf(buf+i, 0);
    i += writeFloatToBuf(buf+i, center[0]);
    i += writeFloatToBuf(buf+i, center[1]);
    i += writeFloatToBuf(buf+i, radius);
    i += writeFloatToBuf(buf+i, thickness);
    i += writeColorToBuf(buf+i, color, 3);
    i += writeStringToBuf(buf+i, setName);

    return buf;
}

unsigned char* RVSender::newLine(const float* a, const float* b, float thickness,
                                 const float* color, const string* setName, int* bufSize) {

    *bufSize = 48 + ((setName != NULL) ? setName->length() : 0);
    unsigned char* buf = new unsigned char[*bufSize];

    long i = 0;
    i += writeCharToBuf(buf+i, 1);
    i += writeCharToBuf(buf+i, 1);
    i += writeFloatToBuf(buf+i, a[0]);
    i += writeFloatToBuf(buf+i, a[1]);
    i += writeFloatToBuf(buf+i, a[2]);
    i += writeFloatToBuf(buf+i, b[0]);
    i += writeFloatToBuf(buf+i, b[1]);
    i += writeFloatToBuf(buf+i, b[2]);
    i += writeFloatToBuf(buf+i, thickness);
    i += writeColorToBuf(buf+i, color, 3);
    i += writeStringToBuf(buf+i, setName);

    return buf;
}

unsigned char* RVSender::newPoint(const float* p, float size, const float* color,
                                  const string* setName, int* bufSize) {

    *bufSize = 30 + ((setName != NULL) ? setName->length() : 0);
    unsigned char* buf = new unsigned char[*bufSize];

    long i = 0;
    i += writeCharToBuf(buf+i, 1);
    i += writeCharToBuf(buf+i, 2);
    i += writeFloatToBuf(buf+i, p[0]);
    i += writeFloatToBuf(buf+i, p[1]);
    i += writeFloatToBuf(buf+i, p[2]);
    i += writeFloatToBuf(buf+i, size);
    i += writeColorToBuf(buf+i, color, 3);
    i += writeStringToBuf(buf+i, setName);

    return buf;
}

unsigned char* RVSender::newSphere(const float* p, float radius, const float* color,
                                   const string* setName, int* bufSize) {

    *bufSize = 30 + ((setName != NULL) ? setName->length() : 0);
    unsigned char* buf = new unsigned char[*bufSize];

    long i = 0;
    i += writeCharToBuf(buf+i, 1);
    i += writeCharToBuf(buf+i, 3);
    i += writeFloatToBuf(buf+i, p[0]);
    i += writeFloatToBuf(buf+i, p[1]);
    i += writeFloatToBuf(buf+i, p[2]);
    i += writeFloatToBuf(buf+i, radius);
    i += writeColorToBuf(buf+i, color, 3);
    i += writeStringToBuf(buf+i, setName);

    return buf;
}

unsigned char* RVSender::newPolygon(const float* v, int numVerts, const float* color,
                                    const string* setName, int* bufSize) {

    *bufSize = 18 * numVerts + 8 + ((setName != NULL) ? setName->length() : 0);
    unsigned char* buf = new unsigned char[*bufSize];

    long i = 0;
    i += writeCharToBuf(buf+i, 1);
    i += writeCharToBuf(buf+i, 4);
    i += writeCharToBuf(buf+i, numVerts);
    i += writeColorToBuf(buf+i, color, 4);

    for (int j = 0; j < numVerts; j++) {
        i += writeFloatToBuf(buf+i, v[j*3+0]);
        i += writeFloatToBuf(buf+i, v[j*3+1]);
        i += writeFloatToBuf(buf+i, v[j*3+2]);
    }

    i += writeStringToBuf(buf+i, setName);

    return buf;
}

unsigned char* RVSender::newAnnotation(const string *txt, const float *p, const float* color,
                                       const string* setName, int* bufSize) {

    *bufSize = 25 + ((setName != NULL) ? setName->length() : 0)
               + ((txt != NULL) ? txt->length(): 0);
    unsigned char* buf = new unsigned char[*bufSize];

    long i = 0;
    i += writeCharToBuf(buf+i, 2);
    i += writeCharToBuf(buf+i, 0);
    i += writeFloatToBuf(buf+i, p[0]);
    i += writeFloatToBuf(buf+i, p[1]);
    i += writeFloatToBuf(buf+i, p[2]);
    i += writeColorToBuf(buf+i, color, 3);
    i += writeStringToBuf(buf+i, txt);
    i += writeStringToBuf(buf+i, setName);

    return buf;
}

unsigned char* RVSender::newAgentAnnotation(const string *txt, const char teamAgent, const float* color, int* bufSize) {

    *bufSize = 7 + ((txt != NULL) ? txt->length(): 0);
    unsigned char* buf = new unsigned char[*bufSize];

    long i = 0;
    i += writeCharToBuf(buf+i, 2);
    i += writeCharToBuf(buf+i, 1);
    i += writeCharToBuf(buf+i, teamAgent);
    i += writeColorToBuf(buf+i, color, 3);
    i += writeStringToBuf(buf+i, txt);

    return buf;
}

unsigned char* RVSender::newRemoveAgentAnnotation(const char teamAgent, int* bufSize) {

    *bufSize = 3;
    unsigned char* buf = new unsigned char[*bufSize];

    long i = 0;
    i += writeCharToBuf(buf+i, 2);
    i += writeCharToBuf(buf+i, 2);
    i += writeCharToBuf(buf+i, teamAgent);

    return buf;
}

unsigned char* RVSender::newSelectAgent(const char teamAgent, int* bufSize) {

    *bufSize = 3;
    unsigned char* buf = new unsigned char[*bufSize];

    long i = 0;
    i += writeCharToBuf(buf+i, 3);
    i += writeCharToBuf(buf+i, 0);
    i += writeCharToBuf(buf+i, teamAgent);

    return buf;
}

/*
int main()
 {
   RVSender send = RVSender();
   string str = string("thing");
   send.drawPoint(1, 1, 1, 4, 1, 0, 0, &str);
   send.swapBuffers(&str);
 }
//*/

