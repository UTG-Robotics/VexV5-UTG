#pragma once
#include "main.h"
#include <string>
#include <vector>

struct Note
{
    int hz;
    int duration;
};

class Sound
{
public:
    Sound(std::string rtttlIn);
    std::vector<Note> getNotes();
    std::vector<Note> notes;

private:
    std::string rtttl;
};