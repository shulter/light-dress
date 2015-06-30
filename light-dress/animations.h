#ifndef __ANIMATIONS_H__
#define __ANIMATIONS_H__

/**
 * To create an animation add an array of struct frameStc:
 * 
 * frameStc animation_name[] = {
 *  { LED1_PWM, LED2_PWM, DELAY },  // frame 1 
 *  { LED1_PWM, LED2_PWM, DELAY },  // frame 2
 *  ....                            
 *  { LED1_PWM, LED2_PWM, DELAY },  // frame n
 * };
 * 
 * The last frame values will be maintained so if the lights, e.g. if the lights need to be off after the
 * animation make sure the last frame is { 0, 0, 0 }
 * 
 * In the main code add an entry (although not strictly necessary if animations.h is included directly)
 * extern frameStc animation_name[];
 * 
 * set the animation frames pointer and number of frames:
 * animation.frames = (frameStc*)animation_name;
 * animation.num_frames = sizeof(animation_name) / sizeof(frameStc);
 * 
 */

typedef struct frameStc {
  uint8_t led1;                   /** PWM value LED1: 0-255 */
  uint8_t led2;                   /** PWM value LED2: 0-255 */
  long delay;                     /** Delay in ms. to wait after LED1 and LED2 values are set */
} frameStc;

typedef struct animationStc {
  int frame;                      /** current frame */
  int num_frames;                 /** total number of frames */
  int frame_delay;                /** current frame delay */
  int repeat;                     /** repeat animation repeat times 0 - MAXINT */
  frameStc *frames;               /** pointer to animation frames array of type struct frameStc */
  unsigned long start_millis;     /** time in ms. since last frame */
} animationStc;

frameStc anim_zone_2_frames[] = {
  {0, 0, 50},
  {255, 255, 500},
  {0, 255, 250},
  {255, 0, 250},
  {0, 0, 100},
  {255, 0, 50},
  {0, 255, 50},
  {255, 0, 50},
  {0, 255, 10},
  {0, 254, 10},
  {0, 253, 10},
  {0, 252, 10},
  {0, 251, 10},
  {0, 250, 10},
  {0, 249, 10},
  {0, 248, 10},
  {0, 247, 10},
  {0, 246, 10},
  {0, 245, 10},
  {0, 244, 10},
  {0, 243, 10},
  {0, 242, 10},
  {0, 241, 10},
  {0, 240, 10},
  {0, 239, 10},
  {0, 238, 10},
  {0, 237, 10},
  {0, 236, 10},
  {0, 235, 10},
  {0, 234, 10},
  {0, 233, 10},
  {0, 232, 10},
  {0, 231, 10},
  {0, 230, 10},
  {0, 229, 10},
  {0, 228, 10},
  {0, 227, 10},
  {0, 226, 10},
  {0, 225, 10},
  {0, 224, 10},
  {0, 223, 10},
  {0, 222, 10},
  {0, 221, 10},
  {0, 220, 10},
  {0, 219, 10},
  {0, 218, 10},
  {0, 217, 10},
  {0, 216, 10},
  {0, 215, 10},
  {0, 214, 10},
  {0, 213, 10},
  {0, 212, 10},
  {0, 211, 10},
  {0, 210, 10},
  {0, 209, 10},
  {0, 208, 10},
  {0, 207, 10},
  {0, 206, 10},
  {0, 205, 10},
  {0, 204, 10},
  {0, 203, 10},
  {0, 202, 10},
  {0, 201, 10},
  {0, 200, 10},
  {0, 199, 10},
  {0, 198, 10},
  {0, 197, 10},
  {0, 196, 10},
  {0, 195, 10},
  {0, 194, 10},
  {0, 193, 10},
  {0, 192, 10},
  {0, 191, 10},
  {0, 190, 10},
  {0, 189, 10},
  {0, 188, 10},
  {0, 187, 10},
  {0, 186, 10},
  {0, 185, 10},
  {0, 184, 10},
  {0, 183, 10},
  {0, 182, 10},
  {0, 181, 10},
  {0, 180, 10},
  {0, 179, 10},
  {0, 178, 10},
  {0, 177, 10},
  {0, 176, 10},
  {0, 175, 10},
  {0, 174, 10},
  {0, 173, 10},
  {0, 172, 10},
  {0, 171, 10},
  {0, 170, 10},
  {0, 169, 10},
  {0, 168, 10},
  {0, 167, 10},
  {0, 166, 10},
  {0, 165, 10},
  {0, 164, 10},
  {0, 163, 10},
  {0, 162, 10},
  {0, 161, 10},
  {0, 160, 10},
  {0, 159, 10},
  {0, 158, 10},
  {0, 157, 10},
  {0, 156, 10},
  {0, 155, 10},
  {0, 154, 10},
  {0, 153, 10},
  {0, 152, 10},
  {0, 151, 10},
  {0, 150, 10},
  {0, 149, 10},
  {0, 148, 10},
  {0, 147, 10},
  {0, 146, 10},
  {0, 145, 10},
  {0, 144, 10},
  {0, 143, 10},
  {0, 142, 10},
  {0, 141, 10},
  {0, 140, 10},
  {0, 139, 10},
  {0, 138, 10},
  {0, 137, 10},
  {0, 136, 10},
  {0, 135, 10},
  {0, 134, 10},
  {0, 133, 10},
  {0, 132, 10},
  {0, 131, 10},
  {0, 130, 10},
  {0, 129, 10},
  {0, 128, 10},
  {0, 127, 10},
  {0, 126, 10},
  {0, 125, 10},
  {0, 124, 10},
  {0, 123, 10},
  {0, 122, 10},
  {0, 121, 10},
  {0, 120, 10},
  {0, 119, 10},
  {0, 118, 10},
  {0, 117, 10},
  {0, 116, 10},
  {0, 115, 10},
  {0, 114, 10},
  {0, 113, 10},
  {0, 112, 10},
  {0, 111, 10},
  {0, 110, 10},
  {0, 109, 10},
  {0, 108, 10},
  {0, 107, 10},
  {0, 106, 10},
  {0, 105, 10},
  {0, 104, 10},
  {0, 103, 10},
  {0, 102, 10},
  {0, 101, 10},
  {0, 100, 10},
  {0, 99, 10},
  {0, 98, 10},
  {0, 97, 10},
  {0, 96, 10},
  {0, 95, 10},
  {0, 94, 10},
  {0, 93, 10},
  {0, 92, 10},
  {0, 91, 10},
  {0, 90, 10},
  {0, 89, 10},
  {0, 88, 10},
  {0, 87, 10},
  {0, 86, 10},
  {0, 85, 10},
  {0, 84, 10},
  {0, 83, 10},
  {0, 82, 10},
  {0, 81, 10},
  {0, 80, 10},
  {0, 79, 10},
  {0, 78, 10},
  {0, 77, 10},
  {0, 76, 10},
  {0, 75, 10},
  {0, 74, 10},
  {0, 73, 10},
  {0, 72, 10},
  {0, 71, 10},
  {0, 70, 10},
  {0, 69, 10},
  {0, 68, 10},
  {0, 67, 10},
  {0, 66, 10},
  {0, 65, 10},
  {0, 64, 10},
  {0, 63, 10},
  {0, 62, 10},
  {0, 61, 10},
  {0, 60, 10},
  {0, 59, 10},
  {0, 58, 10},
  {0, 57, 10},
  {0, 56, 10},
  {0, 55, 10},
  {0, 54, 10},
  {0, 53, 10},
  {0, 52, 10},
  {0, 51, 10},
  {0, 50, 10},
  {0, 49, 10},
  {0, 48, 10},
  {0, 47, 10},
  {0, 46, 10},
  {0, 45, 10},
  {0, 44, 10},
  {0, 43, 10},
  {0, 42, 10},
  {0, 41, 10},
  {0, 40, 10},
  {0, 39, 10},
  {0, 38, 10},
  {0, 37, 10},
  {0, 36, 10},
  {0, 35, 10},
  {0, 34, 10},
  {0, 33, 10},
  {0, 32, 10},
  {0, 31, 10},
  {0, 30, 10},
  {0, 29, 10},
  {0, 28, 10},
  {0, 27, 10},
  {0, 26, 10},
  {0, 25, 10},
  {0, 24, 10},
  {0, 23, 10},
  {0, 22, 10},
  {0, 21, 10},
  {0, 20, 10},
  {0, 19, 10},
  {0, 18, 10},
  {0, 17, 10},
  {0, 16, 10},
  {0, 15, 10},
  {0, 14, 10},
  {0, 13, 10},
  {0, 12, 10},
  {0, 11, 10},
  {0, 10, 10},
  {0, 9, 10},
  {0, 8, 10},
  {0, 7, 10},
  {0, 6, 10},
  {0, 5, 10},
  {0, 4, 10},
  {0, 3, 10},
  {0, 2, 10},
  {0, 1, 10},
  {0, 0, 1000},
};

frameStc anim_quick_fade_frames[] = {
{ 0, 0, 33 },
{ 2, 2, 33 },
{ 4, 4, 33 },
{ 6, 6, 33 },
{ 8, 8, 33 },
{ 10, 10, 33 },
{ 12, 12, 33 },
{ 14, 14, 33 },
{ 16, 16, 33 },
{ 18, 18, 33 },
{ 20, 20, 33 },
{ 22, 22, 33 },
{ 24, 24, 33 },
{ 26, 26, 33 },
{ 28, 28, 33 },
{ 30, 30, 33 },
{ 32, 32, 33 },
{ 34, 34, 33 },
{ 36, 36, 33 },
{ 38, 38, 33 },
{ 40, 40, 33 },
{ 42, 42, 33 },
{ 44, 44, 33 },
{ 46, 46, 33 },
{ 48, 48, 33 },
{ 50, 50, 33 },
{ 52, 52, 33 },
{ 54, 54, 33 },
{ 56, 56, 33 },
{ 58, 58, 33 },
{ 60, 60, 33 },
{ 62, 62, 33 },
{ 64, 64, 33 },
{ 66, 66, 33 },
{ 68, 68, 33 },
{ 70, 70, 33 },
{ 72, 72, 33 },
{ 74, 74, 33 },
{ 76, 76, 33 },
{ 78, 78, 33 },
{ 80, 80, 33 },
{ 82, 82, 33 },
{ 84, 84, 33 },
{ 86, 86, 33 },
{ 88, 88, 33 },
{ 90, 90, 33 },
{ 92, 92, 33 },
{ 94, 94, 33 },
{ 96, 96, 33 },
{ 98, 98, 33 },
{ 100, 100, 33 },
{ 102, 102, 33 },
{ 104, 104, 33 },
{ 106, 106, 33 },
{ 108, 108, 33 },
{ 110, 110, 33 },
{ 112, 112, 33 },
{ 114, 114, 33 },
{ 116, 116, 33 },
{ 118, 118, 33 },
{ 120, 120, 33 },
{ 122, 122, 33 },
{ 124, 124, 33 },
{ 126, 126, 33 },
{ 127, 127, 33 },
{ 124, 124, 33 },
{ 121, 121, 33 },
{ 118, 118, 33 },
{ 115, 115, 33 },
{ 112, 112, 33 },
{ 109, 109, 33 },
{ 106, 106, 33 },
{ 103, 103, 33 },
{ 100, 100, 33 },
{ 97, 97, 33 },
{ 94, 94, 33 },
{ 91, 91, 33 },
{ 88, 88, 33 },
{ 85, 85, 33 },
{ 82, 82, 33 },
{ 79, 79, 33 },
{ 76, 76, 33 },
{ 73, 73, 33 },
{ 70, 70, 33 },
{ 67, 67, 33 },
{ 64, 64, 33 },
{ 61, 61, 33 },
{ 58, 58, 33 },
{ 55, 55, 33 },
{ 52, 52, 33 },
{ 49, 49, 33 },
{ 46, 46, 33 },
{ 43, 43, 33 },
{ 40, 40, 33 },
{ 37, 37, 33 },
{ 34, 34, 33 },
{ 31, 31, 33 },
{ 28, 28, 33 },
{ 25, 25, 33 },
{ 22, 22, 33 },
{ 19, 19, 33 },
{ 16, 16, 33 },
{ 13, 13, 33 },
{ 10, 10, 33 },
{ 7, 7, 33 },
{ 4, 4, 33 },
{ 1, 1, 33 },
{ 0, 0, 33 },
};

frameStc anim_zone_1_frames[] = {
  {0, 255, 10},
  {1, 254, 10},
  {2, 253, 10},
  {3, 252, 10},
  {4, 251, 10},
  {5, 250, 10},
  {6, 249, 10},
  {7, 248, 10},
  {8, 247, 10},
  {9, 246, 10},
  {10, 245, 10},
  {11, 244, 10},
  {12, 243, 10},
  {13, 242, 10},
  {14, 241, 10},
  {15, 240, 10},
  {16, 239, 10},
  {17, 238, 10},
  {18, 237, 10},
  {19, 236, 10},
  {20, 235, 10},
  {21, 234, 10},
  {22, 233, 10},
  {23, 232, 10},
  {24, 231, 10},
  {25, 230, 10},
  {26, 229, 10},
  {27, 228, 10},
  {28, 227, 10},
  {29, 226, 10},
  {30, 225, 10},
  {31, 224, 10},
  {32, 223, 10},
  {33, 222, 10},
  {34, 221, 10},
  {35, 220, 10},
  {36, 219, 10},
  {37, 218, 10},
  {38, 217, 10},
  {39, 216, 10},
  {40, 215, 10},
  {41, 214, 10},
  {42, 213, 10},
  {43, 212, 10},
  {44, 211, 10},
  {45, 210, 10},
  {46, 209, 10},
  {47, 208, 10},
  {48, 207, 10},
  {49, 206, 10},
  {50, 205, 10},
  {51, 204, 10},
  {52, 203, 10},
  {53, 202, 10},
  {54, 201, 10},
  {55, 200, 10},
  {56, 199, 10},
  {57, 198, 10},
  {58, 197, 10},
  {59, 196, 10},
  {60, 195, 10},
  {61, 194, 10},
  {62, 193, 10},
  {63, 192, 10},
  {64, 191, 10},
  {65, 190, 10},
  {66, 189, 10},
  {67, 188, 10},
  {68, 187, 10},
  {69, 186, 10},
  {70, 185, 10},
  {71, 184, 10},
  {72, 183, 10},
  {73, 182, 10},
  {74, 181, 10},
  {75, 180, 10},
  {76, 179, 10},
  {77, 178, 10},
  {78, 177, 10},
  {79, 176, 10},
  {80, 175, 10},
  {81, 174, 10},
  {82, 173, 10},
  {83, 172, 10},
  {84, 171, 10},
  {85, 170, 10},
  {86, 169, 10},
  {87, 168, 10},
  {88, 167, 10},
  {89, 166, 10},
  {90, 165, 10},
  {91, 164, 10},
  {92, 163, 10},
  {93, 162, 10},
  {94, 161, 10},
  {95, 160, 10},
  {96, 159, 10},
  {97, 158, 10},
  {98, 157, 10},
  {99, 156, 10},
  {100, 155, 10},
  {101, 154, 10},
  {102, 153, 10},
  {103, 152, 10},
  {104, 151, 10},
  {105, 150, 10},
  {106, 149, 10},
  {107, 148, 10},
  {108, 147, 10},
  {109, 146, 10},
  {110, 145, 10},
  {111, 144, 10},
  {112, 143, 10},
  {113, 142, 10},
  {114, 141, 10},
  {115, 140, 10},
  {116, 139, 10},
  {117, 138, 10},
  {118, 137, 10},
  {119, 136, 10},
  {120, 135, 10},
  {121, 134, 10},
  {122, 133, 10},
  {123, 132, 10},
  {124, 131, 10},
  {125, 130, 10},
  {126, 129, 10},
  {127, 128, 10},
  {128, 127, 10},
  {129, 126, 10},
  {130, 125, 10},
  {131, 124, 10},
  {132, 123, 10},
  {133, 122, 10},
  {134, 121, 10},
  {135, 120, 10},
  {136, 119, 10},
  {137, 118, 10},
  {138, 117, 10},
  {139, 116, 10},
  {140, 115, 10},
  {141, 114, 10},
  {142, 113, 10},
  {143, 112, 10},
  {144, 111, 10},
  {145, 110, 10},
  {146, 109, 10},
  {147, 108, 10},
  {148, 107, 10},
  {149, 106, 10},
  {150, 105, 10},
  {151, 104, 10},
  {152, 103, 10},
  {153, 102, 10},
  {154, 101, 10},
  {155, 100, 10},
  {156, 99, 10},
  {157, 98, 10},
  {158, 97, 10},
  {159, 96, 10},
  {160, 95, 10},
  {161, 94, 10},
  {162, 93, 10},
  {163, 92, 10},
  {164, 91, 10},
  {165, 90, 10},
  {166, 89, 10},
  {167, 88, 10},
  {168, 87, 10},
  {169, 86, 10},
  {170, 85, 10},
  {171, 84, 10},
  {172, 83, 10},
  {173, 82, 10},
  {174, 81, 10},
  {175, 80, 10},
  {176, 79, 10},
  {177, 78, 10},
  {178, 77, 10},
  {179, 76, 10},
  {180, 75, 10},
  {181, 74, 10},
  {182, 73, 10},
  {183, 72, 10},
  {184, 71, 10},
  {185, 70, 10},
  {186, 69, 10},
  {187, 68, 10},
  {188, 67, 10},
  {189, 66, 10},
  {190, 65, 10},
  {191, 64, 10},
  {192, 63, 10},
  {193, 62, 10},
  {194, 61, 10},
  {195, 60, 10},
  {196, 59, 10},
  {197, 58, 10},
  {198, 57, 10},
  {199, 56, 10},
  {200, 55, 10},
  {201, 54, 10},
  {202, 53, 10},
  {203, 52, 10},
  {204, 51, 10},
  {205, 50, 10},
  {206, 49, 10},
  {207, 48, 10},
  {208, 47, 10},
  {209, 46, 10},
  {210, 45, 10},
  {211, 44, 10},
  {212, 43, 10},
  {213, 42, 10},
  {214, 41, 10},
  {215, 40, 10},
  {216, 39, 10},
  {217, 38, 10},
  {218, 37, 10},
  {219, 36, 10},
  {220, 35, 10},
  {221, 34, 10},
  {222, 33, 10},
  {223, 32, 10},
  {224, 31, 10},
  {225, 30, 10},
  {226, 29, 10},
  {227, 28, 10},
  {228, 27, 10},
  {229, 26, 10},
  {230, 25, 10},
  {231, 24, 10},
  {232, 23, 10},
  {233, 22, 10},
  {234, 21, 10},
  {235, 20, 10},
  {236, 19, 10},
  {237, 18, 10},
  {238, 17, 10},
  {239, 16, 10},
  {240, 15, 10},
  {241, 14, 10},
  {242, 13, 10},
  {243, 12, 10},
  {244, 11, 10},
  {245, 10, 10},
  {246, 9, 10},
  {247, 8, 10},
  {248, 7, 10},
  {249, 6, 10},
  {250, 5, 10},
  {251, 4, 10},
  {252, 3, 10},
  {253, 2, 10},
  {254, 1, 10},
  {255, 0, 10},
  {254, 1, 10},
  {253, 2, 10},
  {252, 3, 10},
  {251, 4, 10},
  {250, 5, 10},
  {249, 6, 10},
  {248, 7, 10},
  {247, 8, 10},
  {246, 9, 10},
  {245, 10, 10},
  {244, 11, 10},
  {243, 12, 10},
  {242, 13, 10},
  {241, 14, 10},
  {240, 15, 10},
  {239, 16, 10},
  {238, 17, 10},
  {237, 18, 10},
  {236, 19, 10},
  {235, 20, 10},
  {234, 21, 10},
  {233, 22, 10},
  {232, 23, 10},
  {231, 24, 10},
  {230, 25, 10},
  {229, 26, 10},
  {228, 27, 10},
  {227, 28, 10},
  {226, 29, 10},
  {225, 30, 10},
  {224, 31, 10},
  {223, 32, 10},
  {222, 33, 10},
  {221, 34, 10},
  {220, 35, 10},
  {219, 36, 10},
  {218, 37, 10},
  {217, 38, 10},
  {216, 39, 10},
  {215, 40, 10},
  {214, 41, 10},
  {213, 42, 10},
  {212, 43, 10},
  {211, 44, 10},
  {210, 45, 10},
  {209, 46, 10},
  {208, 47, 10},
  {207, 48, 10},
  {206, 49, 10},
  {205, 50, 10},
  {204, 51, 10},
  {203, 52, 10},
  {202, 53, 10},
  {201, 54, 10},
  {200, 55, 10},
  {199, 56, 10},
  {198, 57, 10},
  {197, 58, 10},
  {196, 59, 10},
  {195, 60, 10},
  {194, 61, 10},
  {193, 62, 10},
  {192, 63, 10},
  {191, 64, 10},
  {190, 65, 10},
  {189, 66, 10},
  {188, 67, 10},
  {187, 68, 10},
  {186, 69, 10},
  {185, 70, 10},
  {184, 71, 10},
  {183, 72, 10},
  {182, 73, 10},
  {181, 74, 10},
  {180, 75, 10},
  {179, 76, 10},
  {178, 77, 10},
  {177, 78, 10},
  {176, 79, 10},
  {175, 80, 10},
  {174, 81, 10},
  {173, 82, 10},
  {172, 83, 10},
  {171, 84, 10},
  {170, 85, 10},
  {169, 86, 10},
  {168, 87, 10},
  {167, 88, 10},
  {166, 89, 10},
  {165, 90, 10},
  {164, 91, 10},
  {163, 92, 10},
  {162, 93, 10},
  {161, 94, 10},
  {160, 95, 10},
  {159, 96, 10},
  {158, 97, 10},
  {157, 98, 10},
  {156, 99, 10},
  {155, 100, 10},
  {154, 101, 10},
  {153, 102, 10},
  {152, 103, 10},
  {151, 104, 10},
  {150, 105, 10},
  {149, 106, 10},
  {148, 107, 10},
  {147, 108, 10},
  {146, 109, 10},
  {145, 110, 10},
  {144, 111, 10},
  {143, 112, 10},
  {142, 113, 10},
  {141, 114, 10},
  {140, 115, 10},
  {139, 116, 10},
  {138, 117, 10},
  {137, 118, 10},
  {136, 119, 10},
  {135, 120, 10},
  {134, 121, 10},
  {133, 122, 10},
  {132, 123, 10},
  {131, 124, 10},
  {130, 125, 10},
  {129, 126, 10},
  {128, 127, 10},
  {127, 128, 10},
  {126, 129, 10},
  {125, 130, 10},
  {124, 131, 10},
  {123, 132, 10},
  {122, 133, 10},
  {121, 134, 10},
  {120, 135, 10},
  {119, 136, 10},
  {118, 137, 10},
  {117, 138, 10},
  {116, 139, 10},
  {115, 140, 10},
  {114, 141, 10},
  {113, 142, 10},
  {112, 143, 10},
  {111, 144, 10},
  {110, 145, 10},
  {109, 146, 10},
  {108, 147, 10},
  {107, 148, 10},
  {106, 149, 10},
  {105, 150, 10},
  {104, 151, 10},
  {103, 152, 10},
  {102, 153, 10},
  {101, 154, 10},
  {100, 155, 10},
  {99, 156, 10},
  {98, 157, 10},
  {97, 158, 10},
  {96, 159, 10},
  {95, 160, 10},
  {94, 161, 10},
  {93, 162, 10},
  {92, 163, 10},
  {91, 164, 10},
  {90, 165, 10},
  {89, 166, 10},
  {88, 167, 10},
  {87, 168, 10},
  {86, 169, 10},
  {85, 170, 10},
  {84, 171, 10},
  {83, 172, 10},
  {82, 173, 10},
  {81, 174, 10},
  {80, 175, 10},
  {79, 176, 10},
  {78, 177, 10},
  {77, 178, 10},
  {76, 179, 10},
  {75, 180, 10},
  {74, 181, 10},
  {73, 182, 10},
  {72, 183, 10},
  {71, 184, 10},
  {70, 185, 10},
  {69, 186, 10},
  {68, 187, 10},
  {67, 188, 10},
  {66, 189, 10},
  {65, 190, 10},
  {64, 191, 10},
  {63, 192, 10},
  {62, 193, 10},
  {61, 194, 10},
  {60, 195, 10},
  {59, 196, 10},
  {58, 197, 10},
  {57, 198, 10},
  {56, 199, 10},
  {55, 200, 10},
  {54, 201, 10},
  {53, 202, 10},
  {52, 203, 10},
  {51, 204, 10},
  {50, 205, 10},
  {49, 206, 10},
  {48, 207, 10},
  {47, 208, 10},
  {46, 209, 10},
  {45, 210, 10},
  {44, 211, 10},
  {43, 212, 10},
  {42, 213, 10},
  {41, 214, 10},
  {40, 215, 10},
  {39, 216, 10},
  {38, 217, 10},
  {37, 218, 10},
  {36, 219, 10},
  {35, 220, 10},
  {34, 221, 10},
  {33, 222, 10},
  {32, 223, 10},
  {31, 224, 10},
  {30, 225, 10},
  {29, 226, 10},
  {28, 227, 10},
  {27, 228, 10},
  {26, 229, 10},
  {25, 230, 10},
  {24, 231, 10},
  {23, 232, 10},
  {22, 233, 10},
  {21, 234, 10},
  {20, 235, 10},
  {19, 236, 10},
  {18, 237, 10},
  {17, 238, 10},
  {16, 239, 10},
  {15, 240, 10},
  {14, 241, 10},
  {13, 242, 10},
  {12, 243, 10},
  {11, 244, 10},
  {10, 245, 10},
  {9, 246, 10},
  {8, 247, 10},
  {7, 248, 10},
  {6, 249, 10},
  {5, 250, 10},
  {4, 251, 10},
  {3, 252, 10},
  {2, 253, 10},
  {1, 254, 10},
  {0, 0, 1000},
};

frameStc anim_xbee_override_frames[] = {
  {0, 127, 10},
  {124, 3, 10},
  {121, 6, 10},
  {118, 9, 10},
  {115, 12, 10},
  {112, 15, 10},
  {109, 18, 10},
  {106, 21, 10},
  {103, 24, 10},
  {100, 27, 10},
  {97, 30, 10},
  {94, 33, 10},
  {91, 36, 10},
  {88, 39, 10},
  {85, 42, 10},
  {82, 45, 10},
  {79, 48, 10},
  {76, 51, 10},
  {73, 54, 10},
  {70, 57, 10},
  {67, 60, 10},
  {64, 63, 10},
  {61, 66, 10},
  {58, 69, 10},
  {55, 72, 10},
  {52, 75, 10},
  {49, 78, 10},
  {46, 81, 10},
  {43, 84, 10},
  {40, 87, 10},
  {37, 90, 10},
  {34, 93, 10},
  {31, 96, 10},
  {28, 99, 10},
  {25, 102, 10},
  {22, 105, 10},
  {19, 108, 10},
  {16, 111, 10},
  {13, 114, 10},
  {10, 117, 10},
  {7, 120, 10},
  {4, 123, 10},
  {1, 126, 10},
};

frameStc anim_heartbeat_frames[] = {
  {127, 127, 50},
  {0, 0, 500},
  {127, 0, 50},
  {0, 0, 75},
  {0, 127, 50},
  {0, 0, 10},
};

#endif
