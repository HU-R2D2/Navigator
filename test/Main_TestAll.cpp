#include <limits.h>
#include "gtest/gtest.h"

TEST(map, gewoonniets){
    ASSERT_FALSE(true);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    return result;
}
