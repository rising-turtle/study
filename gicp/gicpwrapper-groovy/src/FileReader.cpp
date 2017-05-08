#include "FileReader.h"
#include <iostream>
#include <fstream>
#include "stdlib.h"
#include <dirent.h>

CFileReader::CFileReader():
m_firstnum(-1),
m_lastnum(-1)
{}
CFileReader::~CFileReader()
{}

void CFileReader::getPoseFile(int i, string& f)
{
    if(m_firstnum <0 || m_lastnum < 0)
    {
        cout<<"Dir not set!"<<endl;
        return ;
    }
    if( i < m_firstnum || i > m_lastnum )
    {
        cout<<" index is not valid!"<<endl;
        return;
    }
    f = m_txtSet[i];
}

void CFileReader::getPcdFile(int i, string& f)
{
    if(m_firstnum <0 || m_lastnum < 0)
    {
        cout<<"Dir not set!"<<endl;
        return ;
    }
    if( i < m_firstnum || i > m_lastnum )
    {
        cout<<" index is not valid!"<<endl;
        return;
    }
    f = m_pcdSet[i];
}

void CFileReader::readDir(string dir)
{
    ProcessDirectory(dir);   
    if(m_txtSet.size() > 0 && m_pcdSet.size()>0)
    {
        map<int, string>::reverse_iterator it_max = m_txtSet.rbegin();
        map<int, string>::iterator it_min = m_txtSet.begin();
        m_firstnum = it_min->first;
        m_lastnum = it_max->first;
    }
}


void CFileReader::ProcessDirectory(std::string directory)
{
    std::string dirToOpen = directory;
    auto DIR* dir = opendir(dirToOpen.c_str());

 
    std::cout << "Process directory: " << dirToOpen.c_str() << std::endl;
    if(NULL == dir)
    {
        std::cout << "could not open directory: " << dirToOpen.c_str() << std::endl;
        return;
    }
    
    //set the new path for the content of the directory
    m_path = dirToOpen + "/";

    auto struct dirent* entity = readdir(dir);

    while(entity != NULL)
    {
        ProcessEntity(entity);
        entity = readdir(dir);
    }

    //we finished with the directory so remove it from the path
    // path.resize(path.length() - 1 - directory.length());
    closedir(dir);
}

void CFileReader::ProcessEntity(struct dirent* entity)
{
    //find entity type
    if(entity->d_type == DT_DIR)
    {//it's an direcotry
        //don't process the  '..' and the '.' directories
        if(entity->d_name[0] == '.')
        {
            return;
        }

        //it's an directory so process it
        ProcessDirectory(std::string(entity->d_name));
        return;
    }

    if(entity->d_type == DT_REG)
    {//regular file
        ProcessFile(std::string(entity->d_name));
        return;
    }

    //there are some other types
    //read here http://linux.die.net/man/3/readdir
    std::cout << "Not a file or directory: " << entity->d_name << std::endl;
}


void CFileReader::ProcessFile(std::string file)
{
    // std::cout << "Process file     : " << file.c_str() << std::endl;  
    //if you want to do something with the file add your code here
    size_t split_pos = file.find_last_of(".");
    size_t split_num;
    string suffix = file.substr(split_pos+1);
    string name;
    if(suffix == "txt")
    {
        if((split_num=file.find("pose_"))==0)
        {
            name = file.substr(5, 3);
            // cout<<"insert into txt: "<<name<<endl;
            m_txtSet.insert(make_pair(atoi(name.c_str()), m_path+file));
        }
    }else if(suffix == "pcd")
    {
        if((split_num=file.find("cloud_"))==0)
        {
            name = file.substr(6, 3);
            // cout<<"insert into pcd: "<<name<<endl;
            m_pcdSet.insert(make_pair(atoi(name.c_str()), m_path+file));
        }
    }
}

void CFileReader::Process2Record(string outf)
{
    ofstream ouf(outf.c_str());
    map<int, string>::reverse_iterator it_max = m_txtSet.rbegin();
    map<int, string>::iterator it_min = m_txtSet.begin();
    cout<<"the [min,max] num is: "<<it_min->first<<","<<it_max->first<<endl;
    for(int i=it_min->first; i<=it_max->first; i++)
    {
        ouf<<m_pcdSet[i]<<endl;
        ouf<<m_txtSet[i]<<endl;
    }
/*  map<int, string>::iterator it_ = m_txtSet.begin();
    map<int, string>::iterator it_2;
    while(it_!=m_txtSet.end())
    {
        if((it_2 = m_pcdSet.find(it_->first))== m_pcdSet.end())
        {
            // cout<<"Error!"<<endl;
        }
        ouf<<it_->second<<endl;
        ouf<<it_2->second<<endl;
        it_++;
    }
*/
}
