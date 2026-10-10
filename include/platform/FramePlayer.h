#pragma once
/* ---------------------------------------------------------------------------
 * FramePlayer  --  play numbered PNG frames as a clip.
 *
 * path_format is a printf pattern with one integer, for example
 * "/usd/splash/frame_%04d.png". On the host build the "/usd/" prefix is
 * removed, same as every other robot file, so those frames are read from
 * the working directory.
 *
 * The call blocks until the clip ends. On the V5 the frames go to the brain
 * LCD. On the host they go to the brain screen in the main window. A brain
 * touch, a click on that screen, or Escape skips the rest. Nothing is drawn
 * when no frame file can be opened, so a robot with an empty card still boots.
 * ------------------------------------------------------------------------- */

struct FrameSequence {
    const char* path_format;  // printf pattern, one %d / %u index
    int first_frame;          // file index of the first source frame
    int frame_count;          // source indices to walk; 0 stops at the first missing file
    int frame_step;           // show one file, then skip this many minus one (1 = every file)
    double fps;               // rate of the frames that are actually shown
};

// Returns how many frames were shown.
int playFrameSequence(const FrameSequence& seq);
