#ifndef GENERAL_FNCS_H_
#define GENERAL_FNCS_H_

// C++ Library
#include <ctime>
#include <queue>
#include <vector>

// Linux Library
#include <sys/time.h>

namespace GeneralFncs {

    //! Deletes and erases each dynamically created element
    template<typename T>
    inline void destroyAllPtrsInQueue(std::queue<T*>& q) {
        while (!q.empty()) {
            if (q.front() != NULL) {
                delete q.front();
                q.front() = NULL;
            }
            q.pop();
        }
    }

    //! Deletes and erases each dynamically created element
    template<typename T>
    inline void destroyAllPtrsInVector(std::vector<T*>& vec) {
        while (!vec.empty()) {
            if (vec[0] != NULL) {
                delete vec[0];
                vec[0] = NULL;
            }
            vec.erase(vec.begin());
        }
    }

    //! Deletes and erases a particular index in a vector
    template<typename T>
    inline void destroyPtrInVector(std::vector<T*>& vec, std::size_t elementIndex) {
        if (elementIndex < vec.size() && vec[elementIndex] != NULL) {
            delete vec[elementIndex];
            vec[elementIndex] = NULL;
            vec.erase(vec.begin() + elementIndex);
        }
    }

    //! Deletes the memory pointed by a pointer
    template<typename T>
    inline void deletePtr(T*& ptr) {
        if (ptr != NULL) {
            delete ptr;
        }
        ptr = NULL;
    }
    
    //! Returns the time since  in seconds (value has millisecond-precision)
    inline unsigned long getTime() {
        // get the current time of day
        struct timeval cTime;
        gettimeofday(&cTime, NULL);
        // 43 1/2 years in seconds
        static const unsigned long kL43AndHalfYrsInSecs = 1372726281L;
        // calculate the seconds since middle of 2013 ==> Time Since Epoch (secs) - 43.5 years (secs)
        // then multiple answer by 1000 (to have space for millisecond-precision)
        unsigned long secsPortion = (static_cast<unsigned long>(cTime.tv_sec) - kL43AndHalfYrsInSecs);
        secsPortion *= 1000L;
        // calculate milliseconds ==> Microseconds / 1000 (integer division)
        unsigned long milliSecsPortion = (static_cast<unsigned long>(cTime.tv_usec) / 1000L);
        // add the two parts together and return the answer
        return (secsPortion + milliSecsPortion);
    }

    //! Returns a subset of a vector from the start position up to, but not including, the end position
    template <typename T>
    inline std::vector<T> subVector(const std::vector<T>& vec,
                             const std::size_t startPos,
                             const std::size_t endPos)
    {
        std::vector<T> subVec;
        const std::size_t kSize = vec.size();
        if (startPos >= kSize || endPos > kSize || startPos >= endPos)
            return subVec;
        for (std::size_t pos = startPos; pos < endPos; pos++)
            subVec.push_back(vec[pos]);
        return subVec;
    }
}

#endif // GENERAL_FNCS_H_