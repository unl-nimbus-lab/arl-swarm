

#ifndef HYPER_RECTANGLE
#define HYPER_RECTANGLE

#include "./interval.h"
#include <vector>
#include <algorithm>
#include "json.hpp"


    class hyperRectangle {
        public:
        /* Orthotope used for computing reachable drone states */
            hyperRectangle( std::vector<interval>& in_intervals) {
                dimensions = in_intervals.size();
                faces = 2 * in_intervals.size();
                intervals = in_intervals;
            }

            hyperRectangle() {
                //defualt custrunctor leave me alone;
            }

            int getFaces() {
                /* Total number of faces in the hyperRectangle */
                return faces;
            }

            int getDimensions() {
                /* Total number of dimensions in the hyperRectangle */
                return dimensions;
            }

            std::vector<interval> getIntervals() {
                /* Intervals in the hyperRectangle */
                return intervals;
            }

            float maxWidth() {
                /* get the maximum interval width */

                float max = 0.0;
                float tempMax = 0.0;

                for (int i = 0; i < intervals.size(); i++) {
                    tempMax = intervals[i].max - intervals[i].min;
                    if (tempMax > max)
                        max = tempMax;
                }

                return max;
            }

            bool contains(hyperRectangle inside) {
                /* determine if this hyperRectangle contains the given hyperRectangle */
                if (dimensions != inside.getDimensions())
                    std::cout << "dimension mismatch" << std::endl;

                for (int i = 0; i < intervals.size(); i++ ) {
                    if ( inside.intervals[i].min < intervals[i].min || inside.intervals[i].max > intervals[i].max )
                        return false;
                }

                return true;
            }

            void convexHull(hyperRectangle& rect, bool inPlace = true) {
                if (dimensions != rect.dimensions) {
                    std::cout << "Dimension mismatch" << std::endl;
                }

                hyperRectangle resultRect;

                if (inPlace == false) {
                    resultRect = *this;
                }
                
                for(int dim = 0; dim < dimensions; dim++) {
                    if (rect.intervals[dim].min < intervals[dim].min ) {
                        if (inPlace) {
                            intervals[dim].min = rect.intervals[dim].min;
                        } else {
                            resultRect.intervals[dim].min = rect.intervals[dim].min;
                        }
                    }

                    if ( rect.intervals[dim].max > intervals[dim].max ) {
                        if (inPlace) {
                            intervals[dim].max = rect.intervals[dim].max;
                        } else {
                            resultRect.intervals[dim].max = rect.intervals[dim].max;
                        }
                    }
                }
            }

            bool intersects(hyperRectangle rect) {
                /* Check if this rectangle intersects another rectangle */

                for(int dim = 0; dim < dimensions; dim++) {
                    if (rect.intervals[dim].min > intervals[dim].max || rect.intervals[dim].max < intervals[dim].min ) {
                        return false;
                    }
                }

                return true;
            }

            void disp() {
                // Display the hyperRectangle intervals
                for (int i = 0; i < intervals.size(); i++)
                    intervals[i].disp();
            }

            hyperRectangle(hyperRectangle& rect) {
                /* Deep copy constructor */
                intervals = rect.intervals;
                dimensions = rect.dimensions;
                faces = rect.faces;
            }

            void toJson(nlohmann::json& j, int& i) {
                j[i]["Dimension"] = dimensions;
                for (int dim = 0; dim < dimensions; dim++) {
                    j[i]["Intervals"][dim] =
                        {intervals[dim].min, intervals[dim].max};
                }
                i++;
            }

            std::vector<interval> intervals;
            int dimensions;
            int faces;
    };

#endif