#ifndef SAFETY_CHECKER
#define SAFETY_CHECKER

#include "./hyperRectangle.h"
#include "./interval.h"
#include <ctime>
#include <tuple>
#include <chrono>

class safetyChecker {
    public:
    /* Real-time reachability analysis for collision prevention.

    This implementation is inspired by the following source:

    Tran, H. D., Nguyen, L. V., Musau, P., Xiang, W., & Johnson, T. T. (2019, June).
    Decentralized real-time safety verification for distributed cyber-physical systems.
    In International Conference on Formal Techniques for Distributed Objects,
    Components, and Systems (pp. 261-277). Springer, Cham. */


    hyperRectangle makeNeiborhoodRectangle(
        hyperRectangle original, 
        hyperRectangle bloated, 
        int face, 
        float neiborhoodWidth) {
            
        /* Create a face's neihborhood given a target length */
        hyperRectangle result = bloated;

        int dim = face / 2;

        if (face % 2 == 0) {
            result.intervals[dim].min = original.intervals[dim].min;
            result.intervals[dim].max = original.intervals[dim].min ;
        } else {
            result.intervals[dim].min = original.intervals[dim].max;
            result.intervals[dim].max = original.intervals[dim].max;
        }

        if (neiborhoodWidth < 0) {
            result.intervals[dim].min = result.intervals[dim].min + neiborhoodWidth;
        } else {
            result.intervals[dim].max = result.intervals[dim].max + neiborhoodWidth;
        }

        return result;
    }

    float computeDerivativeBounds(
        hyperRectangle rect, 
        int face, 
        float acceleration[3] ) {
        /* Compute position bounds using simplified quadrotor motion dynamics */
        int dimension = face / 2;

        interval bounds(0,0);

        // If the dimension is position, use the velocity as the bounds
        if (dimension == 0) {
            bounds.min = (rect.intervals[3].min );
            bounds.max = (rect.intervals[3].max );
        } else if (dimension == 1) {
            bounds.min = ( rect.intervals[4].min );
            bounds.max = ( rect.intervals[4].max );
        } else if (dimension == 2) {
            bounds.min = (rect.intervals[5].min);
            bounds.max = (rect.intervals[5].max);
        } else if (dimension == 3) {
            bounds.min = (acceleration[0]);
            bounds.max = (acceleration[0]);
        } else if (dimension == 4) {
            bounds.min = (acceleration[1]);
            bounds.max = (acceleration[1]);
        } else if (dimension == 5) {
            bounds.min = (acceleration[2]);
            bounds.max = (acceleration[2]);
        }

        if (face % 2 == 0) {
            return bounds.min;
        } else {
            return bounds.max;
        }
    }

    std::tuple<hyperRectangle, float> singleFaceLift(
        hyperRectangle rect, 
        float acceleration[3], 
        float stepSize, 
        float timeRemaining) {
        /* Perform a single face lifiting operation */

        hyperRectangle bloatedRect = rect;
        float neighborhoodWidths[rect.getFaces()];
        float derivatives[rect.getFaces()];
        memset(neighborhoodWidths, 0, sizeof(neighborhoodWidths));
        memset(derivatives, 0, sizeof(derivatives));

        bool recompute = true;
        float minNeighborCrossTime;

        while (recompute) {
            recompute = false;
            minNeighborCrossTime = 1000000; //very large

            for (int face = 0; face < rect.getFaces(); face++) {
                //Get the target dimension
                int dimension = face / 2;

                //Determine whether the face is a minimum face or a maximum face
                bool isMin = ((face % 2 ) == 0);

                // Compute the candidate neighborhood
                hyperRectangle neighborRect = bloatedRect;
                neighborRect = ( makeNeiborhoodRectangle(rect, bloatedRect, face, neighborhoodWidths[face]) );

                // Compute the face derivative
                float derivative = computeDerivativeBounds( neighborRect, face, acceleration );

                // calculate the face widths
                float prevNeighborhoodWidth = neighborhoodWidths[face];
                float updatedNeighborhoodWidth = derivative * stepSize;

                // Check if the face width grew outward
                bool prevGrewOutward = (isMin && prevNeighborhoodWidth < 0) || (isMin == false && prevNeighborhoodWidth > 0);
                bool grewOutward = (isMin && updatedNeighborhoodWidth < 0) || (isMin == false && updatedNeighborhoodWidth > 0);

                //Prevent flipping from outward face to inward face
                if (grewOutward == false && prevGrewOutward == true) {
                    updatedNeighborhoodWidth = 0.0;
                    derivative = 0.0;
                }

                //Recompute if flipping from inward to outward or if the derivative doubled
                if ( (prevGrewOutward == false && grewOutward) || 
                    ( abs(updatedNeighborhoodWidth) > 2 * abs(prevNeighborhoodWidth) ) ) {
                    recompute = true;
                }

                //Adjust the bloated rectangle
                if (recompute) {
                    neighborhoodWidths[face] = updatedNeighborhoodWidth;

                    if (isMin && updatedNeighborhoodWidth < 0) {
                        bloatedRect.intervals[dimension].min = (rect.intervals[dimension].min + updatedNeighborhoodWidth );
                    } else if ( (isMin == false) && (updatedNeighborhoodWidth > 0) ) {
                        bloatedRect.intervals[dimension].max = (rect.intervals[dimension].max + updatedNeighborhoodWidth );
                    }

                } else {

                    if ( (derivative < 0 && prevNeighborhoodWidth > 0) || (derivative > 0 && prevNeighborhoodWidth < 0) ) {
                        derivative = 0;
                    }

                    if (derivative != 0) {
                        //std::cout << prevNeighborhoodWidth << " " << derivative << std::endl;
                        float crossTime = prevNeighborhoodWidth / derivative;

                        if (crossTime < minNeighborCrossTime ) {
                            minNeighborCrossTime = crossTime;
                        }
                    }

                    derivatives[face] = derivative;
                }
            }
        }
            
        if ( (minNeighborCrossTime * 2) < stepSize) {
            std::cout << "The minimum neighborhood cross time is less than half of the step size" << std::endl;
        }

        float timeToElapse = minNeighborCrossTime;

        if (timeRemaining < timeToElapse) {
            timeToElapse = timeRemaining;
        }

        //Make a copy of the rectangle to revent accidental modifications to the original
        hyperRectangle rekt = rect;

        //rect.disp();

        //Perform the face lift
        for (int dim = 0; dim < rect.dimensions; dim++) {
            float tempMin = rekt.intervals[dim].min;
            rekt.intervals[dim].min = ( tempMin + derivatives[2 * dim] * timeToElapse);
            //std::cout << derivatives[2* dim] << " " << timeToElapse << std::endl;
            float tempMax = rekt.intervals[dim].max;
            rekt.intervals[dim].max = ( tempMax + derivatives[2 * dim + 1] * timeToElapse);
        }

        if (bloatedRect.contains(rect) == false) {
            std::cout << "Lifted rectangle is outside of teh bloated rectangle" << std::endl;
        }

        //rekt.disp();
        //std::cout << std::endl;

        return {rekt, timeToElapse};
    }

    std::tuple<hyperRectangle, float> faceLiftingIterativeImprovement( 
        hyperRectangle rect, 
        float startTime, 
        float acceleration[3], 
        float reachTime, 
        float initialStepSize = 0.01, 
        float timeOut = 0.01,
        float minStepSize = 0.0000001) {
        
        float stepSize = initialStepSize;

        hyperRectangle hull = rect;
        float endTime = startTime;

        auto start = std::chrono::high_resolution_clock::now();
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();


        while ( (duration / 1000.0) < timeOut) {
            if (stepSize < minStepSize) {
                break;
            }
            float reachTimeRemaining = reachTime;
            float reachTimeAdvance = 0.0;

            while (reachTimeRemaining > 0) {
                auto result = singleFaceLift(rect, acceleration, stepSize, reachTimeRemaining);

                hull.convexHull(rect, true);

                reachTimeAdvance += std::get<1>(result);
                endTime = startTime + 1000 * reachTimeAdvance;

                reachTimeRemaining -= std::get<1>(result);
            }

            stepSize = stepSize / 2.0;

            end = std::chrono::high_resolution_clock::now();
            duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        }

        return {hull, endTime}; 
    }
};

#endif