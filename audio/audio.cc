/*
Optional communication protocol for 2015 RoboCup 3D simulation
drop-in player challenge
*/

#include <iostream>
#include "audio.h"

#include "../headers/Field.h"


using namespace std;

//#define HALF_FIELD_X 15.0
//#define HALF_FIELD_Y 10.0
//#define NUM_AGENTS 11

/*
  These are x and y limits for the ball and the players.
*/
double minBallX = -HALF_FIELD_X - 2.0;
double maxBallX = HALF_FIELD_X + 2.0;
double minBallY = -HALF_FIELD_Y - 2.0;
double maxBallY = HALF_FIELD_Y + 2.0;

double minAgentX = -HALF_FIELD_X - 5.0;
double maxAgentX = HALF_FIELD_X + 5.0;
double minAgentY = -HALF_FIELD_Y - 5.0;
double maxAgentY = HALF_FIELD_Y + 5.0;

// ------ Inteface Methods (call these) ------

/**
 * Creates say message if it currently time for the agent to speak.
 * uNum - Uniform number of agent (1-11)
 * currentServerTime - the current server time, NOT the game time
 * ballLastSeenServerTime - last server time ball was seen, NOT last game time
 * ballX - global X position of ball
 * ballY - global Y position of ball
 * myX - my global X position
 * myY - my global Y position
 * fFallen - whether or not I'm currently fallen
 * message - string to return say message in
 * RETURN - true if message is created, false otherwise
 */
bool makeSayMessage(const int &uNum, const double &currentServerTime, const double &ballLastSeenServerTime, const double &ballX, const double &ballY, const double &myX, const double &myY, const bool &fFallen, string &message) {
    int cycles = int((currentServerTime * 50) + 0.1);
    if (cycles % (NUM_AGENTS*2) != (uNum-1)*2) {
        // Not our time slice turn to say a message
        return false;
    }

    vector<int> bits;
    if(!(dataToBits(currentServerTime, ballLastSeenServerTime, ballX, ballY, myX, myY, fFallen, bits))) {
        return false;
    }

    if(!(bitsToString(bits, message))) {
        return false;
    }

    message = "(say " + message + ")";

    return true;
}

/**
 * Parses message body of hear perceptor (need to first strip off hear header, time, and self/direction).
 * message - message to process
 * heardServerTime, the server time the message was heard, NOT the game time
 * uNum - uniform of the agent who said the message
 * ballLastSeenServerTime - server time agent who said the message saw the ball, NOT the game time
 * ballX - reported global X position of the ball
 * ballY - reported global Y position of the ball
 * agentX - reported global X position of the agent who said the message
 * agentY - reported glboal Y position of the agent who said the message
 * fFallen - whether or not agent who said the message is currently fallen
 * time - time message was said
 * RETURN - true if valid message parsed, false otherwise
 */
bool processHearMessage(const string &message, const double &heardServerTime, int &uNum, double &ballLastSeenServerTime, double &ballX, double &ballY, double &agentX, double &agentY, bool &fFallen, double &time) {

    // Initialize values
    uNum = 0;
    ballLastSeenServerTime = 0;
    ballX = 0;
    ballY = 0;
    agentX = 0;
    agentY = 0;
    fFallen = false;
    time = 0;


    vector<int> bits;
    if(!(stringToBits(message, bits))) {
        return false;
    }

    if(!(bitsToData(bits, time, ballLastSeenServerTime, ballX, ballY, agentX, agentY, fFallen))) {
        return false;
    }

    time += double(int((heardServerTime-time)/1310.72))*1310.72;
    ballLastSeenServerTime +=  double(int((heardServerTime-ballLastSeenServerTime)/1310.72))*1310.72;

    if (heardServerTime-time >= .07 || heardServerTime-time < -.001) {
        return false;
    }

    int cycles = int((time * 50) + 0.1);
    uNum = (cycles%(NUM_AGENTS*2))/2+1;

    return true;
}


//------ Implentation Methods (called by inteface methods) ------

vector<int> intToBits(const int &n, const int &numBits) {

    vector<int> bits;

    if(n < 0) {
        bits.clear();
        return bits;
    }

    int m = n; //Copy.

    bits.resize(numBits);
    for(int i = numBits - 1; i >= 0; i--) {
        bits[i] = m % 2;
        m /= 2;
    }

    return bits;
}

vector<int> intToBits(const unsigned long long &n, const int &numBits) {
    vector<int> bits;

    unsigned long long m = n; //Copy.

    bits.resize(numBits);
    for(int i = numBits - 1; i >= 0; i--) {
        bits[i] = m % 2;
        m /= 2;
    }

    return bits;
}

int bitsToInt(const vector<int> &bits, const int &start, const int &end) {

    if(start < 0 || end >= (int)bits.size()) {
        return 0;//Error.
    }

    int n = 0;
    for(int i = start; i <= end; i++) {
        n *= 2;
        n += bits[i];
    }

    return n;
}

bool dataToBits(const double &time, const double &ballLastSeenTime, const double &ballX, const double &ballY, const double &myX, const double &myY, const bool &fFallen, vector<int> &bits) {

    int cycles = (time * 50) + 0.1;
    cycles = cycles%(1<<16);
    vector<int> timeBits = intToBits(cycles, 16);

    int ballLastSeenCycle = (ballLastSeenTime * 50) + 0.1;
    ballLastSeenCycle = ballLastSeenCycle%(1<<16);
    vector<int> ballLastSeenTimeBits = intToBits(ballLastSeenCycle, 16);

    double clippedBallX = (ballX < minBallX)? minBallX : ((ballX > maxBallX)? maxBallX : ballX);
    int bx = (((clippedBallX - minBallX) * 1023) / (maxBallX - minBallX)) + 0.5;
    vector<int> ballXBits = intToBits(bx, 10);

    double clippedBallY = (ballY < minBallY)? minBallY : ((ballY > maxBallY)? maxBallY : ballY);
    int by = (((clippedBallY - minBallY) * 1023) / (maxBallY - minBallY)) + 0.5;
    vector<int> ballYBits = intToBits(by, 10);

    double clippedMyX = (myX < minAgentX)? minAgentX : ((myX > maxAgentX)? maxAgentX : myX);
    int mx = (((clippedMyX - minAgentX) * 1023) / (maxAgentX - minAgentX)) + 0.5;
    vector<int> myXBits = intToBits(mx, 10);

    double clippedMyY = (myY < minAgentY)? minAgentY : ((myY > maxAgentY)? maxAgentY : myY);
    int my = (((clippedMyY - minAgentY) * 1023) / (maxAgentY - minAgentY)) + 0.5;
    vector<int> myYBits = intToBits(my, 10);

    int fallenBit = (fFallen)? 1 : 0;

    bits.insert(bits.end(), timeBits.begin(), timeBits.end());//16
    bits.insert(bits.end(), ballLastSeenTimeBits.begin(), ballLastSeenTimeBits.end());//16
    bits.insert(bits.end(), ballXBits.begin(), ballXBits.end());//10
    bits.insert(bits.end(), ballYBits.begin(), ballYBits.end());//10
    bits.insert(bits.end(), myXBits.begin(), myXBits.end());//10
    bits.insert(bits.end(), myYBits.begin(), myYBits.end());//10
    bits.push_back(fallenBit);//1

    return true;

}

bool bitsToString(const vector<int> &bits, string &message) {

    message = "";
    if(commAlphabet.size() != 64) {
        cerr << "bitsToString: alphabet size not 64!\n";
        return false;
    }

    vector<int> index;
    index.resize((bits.size() + 5) / 6);
    size_t ctr = 0;
    for(size_t i = 0; i < index.size(); i++) {

        index[i] = 0;
        for(int j = 0; j < 6; j++) {

            index[i] *= 2;

            if(ctr < bits.size()) {
                index[i] += bits[ctr];
                ctr++;
            }

        }
    }

    for(size_t i = 0; i < index.size(); i++) {
        message += commAlphabet.at(index[i]);
    }

    return true;
}


bool bitsToData(const vector<int> &bits, double &time, double &ballLastSeenTime, double &ballX, double &ballY, double &agentX, double &agentY, bool &fFallen) {
    if(bits.size() < (16 + 16 + 10 + 10 + 10 + 10 + 1)) {
        time = 0;
        ballLastSeenTime = 0,
        ballX = 0;
        ballY = 0;
        agentX = 0;
        agentY = 0;
        fFallen = false;
        return false;
    }

    int ctr = 0;

    int cycles = bitsToInt(bits, ctr, ctr + 15);
    time = cycles * 0.02;
    ctr += 16;

    int ballLastSeenCycles = bitsToInt(bits, ctr, ctr + 15);
    ballLastSeenTime = ballLastSeenCycles * 0.02;
    ctr += 16;

    int bx = bitsToInt(bits, ctr, ctr + 9);
    ballX = minBallX + ((maxBallX - minBallX) * (bx / 1023.0));
    ctr += 10;

    int by = bitsToInt(bits, ctr, ctr + 9);
    ballY = minBallY + ((maxBallY - minBallY) * (by / 1023.0));
    ctr += 10;

    int ax = bitsToInt(bits, ctr, ctr + 9);
    agentX = minAgentX + ((maxAgentX - minAgentX) * (ax / 1023.0));
    ctr += 10;

    int ay = bitsToInt(bits, ctr, ctr + 9);
    agentY = minAgentY + ((maxAgentY - minAgentY) * (ay / 1023.0));
    ctr += 10;

    fFallen = (bits[ctr] == 0)? false : true;
    ctr += 1;

    return true;
}


bool stringToBits(const string &message, vector<int> &bits) {

    if(commAlphabet.size() != 64) {
        cerr << "bits2String: alphabet size not 64!\n";
        return false;
    }

    bits.resize(message.length() * 6);

    for(size_t i = 0; i < message.length(); i++) {

        // Make sure every letter in the message comes from our alphabet. Make a mapping.
        // If any violation, return false;
        const char c = message.at(i);
        size_t n = commAlphabet.find(c);
        if(n == string::npos) {
            bits.clear();
            return false;
        }

        for(int j = 5; j >= 0; j--) {
            bits[(i * 6) + j] = n % 2;
            n /= 2;
        }
    }

    return true;
}
