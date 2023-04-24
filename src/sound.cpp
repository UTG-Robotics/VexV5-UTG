#include "main.h"
Sound::Sound(std::string rtttlIn)
{
    this->rtttl = rtttlIn;

    int curChar = 0;
    // Parse Title
    std::string title = "";
    while (rtttl[curChar] != ':')
    {
        title += rtttl[curChar];
        curChar++;
    }
    curChar++;
    // Parse defaults
    int defaultDuration = 4;
    int defaultOctave = 6;
    int BPM = 63;
    while (rtttl[curChar] != ':')
    {
        if (rtttl[curChar] == 'd')
        {
            curChar += 2;
            std::string duration;
            while (rtttl[curChar] != ',' && rtttl[curChar] != ':')
            {
                duration += rtttl[curChar];
                curChar++;
            }
            defaultDuration = std::stoi(duration);
            curChar--;
        }
        else if (rtttl[curChar] == 'o')
        {
            curChar += 2;
            std::string octave;
            while (rtttl[curChar] != ',' && rtttl[curChar] != ':')
            {
                octave += rtttl[curChar];
                curChar++;
            }
            defaultOctave = std::stoi(octave);
            curChar--;
        }
        else if (rtttl[curChar] == 'b')
        {
            curChar += 2;
            std::string bpm;
            while (rtttl[curChar] != ',' && rtttl[curChar] != ':')
            {
                bpm += rtttl[curChar];
                curChar++;
            }
            BPM = std::stoi(bpm);
            curChar--;
        }
        curChar++;
    }
    curChar++;
    // Parse Notes
    float duration = defaultDuration;
    int octave = defaultOctave;
    int note = 0;
    while (curChar < rtttl.length())
    {
        // Parse duration
        if (rtttl[curChar] >= '0' && rtttl[curChar] <= '9')
        {
            std::string durationStr;
            while (rtttl[curChar] >= '0' && rtttl[curChar] <= '9')
            {
                durationStr += rtttl[curChar];
                curChar++;
            }
            duration = std::stoi(durationStr);
        }
        // Parse note
        switch (rtttl[curChar])
        {
        case 'c':
            note = 1;
            break;
        case 'd':
            note = 3;
            break;
        case 'e':
            note = 5;
            break;
        case 'f':
            note = 6;
            break;
        case 'g':
            note = 8;
            break;
        case 'a':
            note = 10;
            break;
        case 'b':
            note = 12;
            break;
        case 'p':
            note = 0;
            break;
        }
        curChar++;
        if (rtttl[curChar] == '#')
        {
            note++;
            curChar++;
        }
        else if (rtttl[curChar] == 'b')
        {
            note--;
            curChar++;
        }
        if (rtttl[curChar] == '.')
        {
            duration /= 1.5;
            curChar++;
        }
        // Parse octave
        if (rtttl[curChar] >= '0' && rtttl[curChar] <= '9')
        {
            octave = rtttl[curChar] - '0';
            curChar++;
        }
        // Parse dot
        if (rtttl[curChar] == '.')
        {
            duration /= 2;
            curChar++;
        }
        // Parse comma
        if (rtttl[curChar] == ',')
        {
            // Add note
            // std::cout << "Note: " << note << " Octave: " << octave << " Duration: " << duration << std::endl;
            Note tmpNote;
            tmpNote.hz = note == 0 ? 0 : 440 * std::pow(2, (note - 9 + (octave - 4) * 12) / 12.0);
            tmpNote.duration = std::round(60000.0 / (double)BPM / (double)duration * 4);
            notes.push_back(tmpNote);
            duration = defaultDuration;
            octave = defaultOctave;
            curChar++;
        }
    }
    std::cout << "Note: " << note << " Octave: " << octave << " Duration: " << duration << std::endl;
    Note tmpNote;
    tmpNote.hz = note == 0 ? 0 : 440 * std::pow(2, (note - 9 + (octave - 4) * 12) / 12.0);
    tmpNote.duration = std::round(60000.0 / (double)BPM / (double)duration);
    notes.push_back(tmpNote);
    // std::cout << defaultDuration << " " << BPM << " " << defaultOctave << std::endl;
}

std::vector<Note> Sound::getNotes()
{
    return notes;
}