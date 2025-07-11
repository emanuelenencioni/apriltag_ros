#include "tag_functions.hpp"

// default tag families
#include <apriltag/tag16h5.h>
#include <apriltag/tag25h9.h>
#include <apriltag/tag36h11.h>
#include <apriltag/tagCircle21h7.h>
#include <apriltag/tagCircle49h12.h>
#include <apriltag/tagCustom48h12.h>
#include <apriltag/tagStandard41h12.h>
#include <apriltag/tagStandard52h13.h>

// create and delete functions for default tags
#define TAG_FUN(name) {#name, {tag##name##_create, tag##name##_destroy}},

// function pointer for tag family creation / destruction
const std::unordered_map<std::string, std::pair<apriltag_family_t* (*) (void), void (*)(apriltag_family_t*)>> tag_fun{
    // clang-format off
    TAG_FUN(36h11)
    TAG_FUN(25h9)
    TAG_FUN(16h5)
    TAG_FUN(Circle21h7)
    TAG_FUN(Circle49h12)
    TAG_FUN(Custom48h12)
    TAG_FUN(Standard41h12)
    TAG_FUN(Standard52h13)
    // clang-format on
};
