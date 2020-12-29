/*
 * Copyright (c) 2011 Mark Liversedge (liversedge@gmail.com)
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc., 51
 * Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef _GC_NSampleSmoothing_h
#define _GC_NSampleSmoothing_h 1

template <size_t N>
class NSampleSmoothing
{
    private:
        int    nSamples   = 0;
        double samples[N];
        int    index      = 0;
        double total      = 0;
        bool   full       = false;

    public:
        NSampleSmoothing()
        {
            reset();
        }

        void reset()
        {
            for (int i=0; i<N; ++i)
                samples[i] = 0.;
            nSamples = 0;
            index    = 0;
            total    = 0;
            full     = false;
        }

        void update(double newVal)
        {
            ++nSamples;

            total += newVal - samples[index];
            samples[index] = newVal;
            if (++index == N) { index = 0; full = true; }
        }

        bool is_full() const
        {
            return full;
        }

        double mean() const
        {
            // average if we have enough values, otherwise return latest
            return total / N;//(nSamples > N) ? (total / N) : samples[(index + (N-1))%N];
        }

        double stddev() const
        {
            const double avg = mean();
            const double sum_squares = std::accumulate(std::begin(samples), std::end(samples), 0.0, [avg](double acc, double sample){return acc + (sample-avg)*(sample-avg);});
            return sqrt(sum_squares / static_cast<double>(N));
        }
};

#endif // _GC_NSampleSmoothing_h

