/*
Optional communication protocol for 2015 RoboCup 3D simulation
drop-in player challenge
*/

#ifndef _AUDIO_H
#define _AUDIO_H

#include <string>
#include <vector>

// Communication alphabet must have 64 symbols.
const std::string commAlphabet =  "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789*#";

/*-------------------------------Utilities---------------------------------------------*/
std::vector<int> intToBits(const int &n, const int &numBits);
std::vector<int> intToBits(const unsigned long long &n, const int &numBits);
int bitsToInt(const std::vector<int> &bits, const int &start, const int &end);

/*-------------------------------Encoding---------------------------------------------*/
bool makeSayMessage(const int &uNum, const double &currentServerTime, const double &ballLastSeenServerTime, const double &ballX, const double &ballY, const double &myX, const double &myY, const bool &fFallen, std::string &message);
bool dataToBits(const double &time, const double &ballLastSeenTime, const double &ballX, const double &ballY, const double &myX, const double &myY, const bool &fFallen, std::vector<int> &bits);
bool bitsToString(const std::vector<int> &bits, std::string &message);

/*-------------------------------Decoding---------------------------------------------*/
bool processHearMessage(const std::string &message, const double &heardServerTime, int &uNum, double &ballLastSeenServerTime, double &ballX, double &ballY, double &agentX, double &agentY, bool &fFallen, double &time);
bool bitsToData(const std::vector<int> &bits, double &time, double &ballLastSeenTime, double &ballX, double &ballY, double &agentX, double &agentY, bool &fFallen);
bool stringToBits(const std::string &message, std::vector<int> &bits);

#endif
