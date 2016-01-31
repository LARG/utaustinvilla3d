/**
 * @file RingBufferWithSum.h
 *
 * Declaration of template class RingBufferWithSum
 *
 * @author Matthias Jüngel
 * @author Tobias Oberlies
 */

#ifndef __RingBufferWithSum_h_
#define __RingBufferWithSum_h_


/**
 * @class RingBufferWithSum
 *
 * Template class for cyclic buffering of the last n values of the type C
 * and with a function that returns the sum of all entries.
 */
template <class C, int n> class RingBufferWithSum
{
public:
    /** Constructor */
    RingBufferWithSum() {
        init();
    }

    /**
     * initializes the RingBufferWithSum
     */
    void init () {
        current = n - 1;
        numberOfEntries = 0;
        sum = C();
    }

    /**
     * adds an entry to the buffer
     * \param value value to be added
     */
    void add (C value)
    {
        if(numberOfEntries == n) sum -= getEntry(numberOfEntries - 1);
        sum += value;
        current++;
        if (current==n) current=0;
        if (++numberOfEntries >= n) numberOfEntries = n;
        buffer[current] = value;
    }

    /**
     * returns an entry
     * \param i index of entry counting from last added (last=0,...)
     * \return a reference to the buffer entry
     */
    C getEntry (int i)
    {
        int j = current - i;
        j %= n;
        if (j < 0) j += n;
        return buffer[j];
    }

    C getSum()
    {
        return sum;
    }

    C getMinimum()
    {
        // Return 0 if buffer is empty
        if (0==numberOfEntries) return C();

        C min = buffer[0];
        for(int i = 0; i < numberOfEntries; i++)
        {
            if(buffer[i] < min) min = buffer[i];
        }
        return min;
    }

    /**
     * returns the average value of all entries
     * \return the average value
     */
    C getAverage()
    {
        // Return 0 if buffer is empty
        if (0==numberOfEntries) return C();

        return (sum / numberOfEntries);
    }

    /**
     * returns an entry
     * \param i index of entry counting from last added (last=0,...)
     * \return a reference to the buffer entry
     */
    C operator[] (int i)
    {
        return getEntry(i);
    }

    /**
     * returns a constant entry.
     * \param i index of entry counting from last added (last=0,...)
     * \return a reference to the buffer entry
     */
    C operator[] (int i) const
    {
        return buffer[i > current ? n + current - i : current - i];
    }

    inline int getNumberOfEntries() const
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
    C buffer[n];
    C sum;
};

#endif // __RingBufferWithSum_h_

