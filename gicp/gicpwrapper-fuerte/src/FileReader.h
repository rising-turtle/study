#ifndef FILE_READER_H
#define FILE_READER_H
#include <string>
#include <map>
using namespace std;

class CFileReader
{
public:
    CFileReader();
    ~CFileReader();
    void readDir(string dir);
    void Process2Record(string outf);
    void getPoseFile(int, string&);
    void getPcdFile(int, string&);
public:
    void ProcessDirectory(std::string directory);
    void ProcessFile(std::string file);
    void ProcessEntity(struct dirent* entity);
    map<int, string> m_txtSet;
    map<int, string> m_pcdSet;
    int m_firstnum;
    int m_lastnum;
    string m_path;
};




#endif
