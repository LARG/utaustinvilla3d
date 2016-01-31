/**
 * @file RingBuffer.h
 *
 * Declaration of class RingBuffer
 *
 * @author Max Risler
 */

#ifndef __RingBuffer_h_
#define __RingBuffer_h_

/**
 * @class RingBuffer
 *
 * template class for cyclic buffering of the last n values of Type V
 */
template <class V, int n> class RingBuffer
{
public:
    /** Constructor */
    RingBuffer() {
        init();
    }

    /**
     * initializes the Ringbuffer
     */
    void init () {
        current = n - 1;
        numberOfEntries = 0;
    }

    /**
     * adds an entry to the buffer
     * \param v value to be added
     */
    void add (const V& v)
    {
        add();
        buffer[current] = v;
    }

    /**
     * adds an entry to the buffer.
     * The new head is not initialized, but can be changed afterwards.
     */
    void add ()
    {
        current++;
        if (current==n) current=0;
        if (++numberOfEntries >= n) numberOfEntries = n;
    }

    /**
     * removes the first added entry to the buffer
     */
    void removeFirst ()
    {
        --numberOfEntries;
    }

    /**
     * returns an entry
     * \param i index of entry counting from last added (last=0,...)
     * \return a reference to the buffer entry
     */
    V& getEntry (int i)
    {
        int j = current - i;
        j %= n;
        if (j < 0) j += n;
        return buffer[j];
    }

    /**
     * returns an const entry
     * \param i index of entry counting from last added (last=0,...)
     * \return a reference to the buffer entry
     */
    const V& getEntry (int i) const
    {
        int j = current - i;
        j %= n;
        if (j < 0) j += n;
        return buffer[j];
    }

    /**
      * returns an entry
      * \param v the value the entry i shall be updated with
      * \param i index of entry counting from last added (last=0,...)
      */
    void updateEntry(const V& v, int i)
    {
        int j = current - i;
        j %= n;
        if (j < 0) j += n;
        buffer[j] = v;
    }

    /**
     * returns an entry
     * \param i index of entry counting from last added (last=0,...)
     * \return a reference to the buffer entry
     */
    V& operator[] (int i)
    {
        return getEntry(i);
    }

    /**
     * returns a constant entry.
     * \param i index of entry counting from last added (last=0,...)
     * \return a reference to the buffer entry
     */
    const V& operator[] (int i) const
    {
        return buffer[i > current ? n + current - i : current - i];
    }

    /** Returns the number of elements that are currently in the ring buffer
    * \return The number
    */
    int getNumberOfEntries() const
    {
        return numberOfEntries;
    }

    /**
    * Returns the maximum entry count.
    * \return The maximum entry count.
    */
    inline int getMaxEntries() const
    {
        return n;
    }

private:
    int current;
    int numberOfEntries;
    V buffer[n];
};

#endif // __RingBuffer_h_
