#ifndef __COMETS_UTILS_FILES__
#define __COMETS_UTILS_FILES__

#include <fcntl.h>
#include <unistd.h>

namespace comets
{
    class unique_fd {
    public:
        static unique_fd open(const char* path, int flags) {
            return unique_fd(open(path, flags));
        }

        unique_fd() = default;
        explicit unique_fd(int fd) : fd(fd) {}
    
        unique_fd(unique_fd &&) = default;
        unique_fd(unique_fd const&) = delete;
        unique_fd& operator=(unique_fd &&) = default;
        unique_fd& operator=(unique_fd const&) = delete;

        ~unique_fd() {
            if (fd > 0) {
                ::close(fd);
                fd = 0;
            }
        }

        auto get() const noexcept -> int {
            return fd;
        }

    private:
        int fd = 0;
    };
} // namespace comets


#endif
