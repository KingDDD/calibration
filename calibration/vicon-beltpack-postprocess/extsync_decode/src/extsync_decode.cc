#include <iostream>
//#include <istream>

//#include <algorithm>
#include <numeric>

//#include <cmath>
#include <cinttypes>
#include <cstdint>
#include <cstring>


#include <array>
#include <vector>
#include <deque>
//#include <string>

#include <boost/optional.hpp>


// Whether or not to display LOG << ... ; output.
// Enable with command line arg: -d [ --debug ]
static bool debug_log = false;

#define LOG if (!debug_log) {} else std::cout


// Element for describing state transitions from external events.
// (See nvidia/linux-4.4.git/include/misc/ml_wearable.h: struct extsync_event)
//  ktime
//    Kernel time on beltpack during event.
//  mtime
//    Minotaur time on beltpack (periodically synchronized with wearable)
//    during event.
//  state
//    New state of external sycn pin after event
//      0 - Falling edge
//      1 - Rising edge
//  event_num
//    ID for events. Should be monotonically increasing. Sequential events
//    should have an event_num difference of 1.

struct Edge {
  uint64_t ktime;
  uint64_t mtime;
  int      state;
  int      event_num;
};


// Easier to deal with a sequence of pulses than edges.
//  high
//    Whether pulse is high or low.
//  duration_us
//    Pulse duration, in microseconds.
struct Pulse {
  bool     high;
  uint64_t duration_us;
  int      event_num_left;
  int      event_num_right;
};


// Decoded trigger event.
struct Trigger {
  uint64_t ktime;
  uint64_t mtime;
  bool     payload_present;
  uint32_t frame_num;
  double   mse;
};



// Pattern element for describing a region of expected pulse behavior.
//  high
//    Whether expected pulse is high or low.
//  min_duration_us, max_duration_us
//    Minimum and maximum pulse duration, in microseconds.
//  weight
//    Importance of this Pattern element, relative to all elements in a filter.
//    Larger values give a higher penalty for pulses outside the min/max
//    duration. Lower values give a smaller penalty. Zero will ignore the range
//    completely.

struct Pattern {
  bool     high;
  uint64_t min_duration_us;
  uint64_t max_duration_us;
  double   weight;
};


// List of Pattern elements defining expected payload structure. This list is
// used as a matched filter template for the incoming Edge stream, in order to
// reliably identify patterns that are most likely to be valid payloads.

static std::vector<Pattern> payload_filter = {{
  { true,  10000, 700000, 1.0 }, // initial trigger edge
  { false, 1000,  16000,  1.0 }, // Payload byte 0 (MSB) high nibble
  { true,  4000,  4000,   1.0 }, // Guard interval
  { false, 1000,  16000,  1.0 }, // Payload byte 0 low nibble
  { true,  4000,  4000,   1.0 },
  { false, 1000,  16000,  1.0 }, // Payload byte 1 high
  { true,  4000,  4000,   1.0 },
  { false, 1000,  16000,  1.0 }, // Payload byte 1 low
  { true,  4000,  4000,   1.0 },
  { false, 1000,  16000,  1.0 }, // Payload byte 2 high
  { true,  4000,  4000,   1.0 },
  { false, 1000,  16000,  1.0 }, // Payload byte 2 low
  { true,  4000,  4000,   1.0 },
  { false, 1000,  16000,  1.0 }, // Payload byte 3 (LSB) high
  { true,  4000,  4000,   1.0 },
  { false, 1000,  16000,  1.0 }, // Payload byte 3 low
  { true,  10000, 10000,  1.0 }, // Stop bit
}};




struct Codeword {
  bool trigger;
  bool nibble;
  uint32_t subtract;
  uint32_t divide;
  uint32_t leftshift;
};

static std::vector<Codeword> payload_decoder = {{
  { true,  false, 0,   0,    0  },
  { false, true,  500, 1000, 28 },
  { false, false, 0,   0,    0  },
  { false, true,  500, 1000, 24 },
  { false, false, 0,   0,    0  },
  { false, true,  500, 1000, 20 },
  { false, false, 0,   0,    0  },
  { false, true,  500, 1000, 16 },
  { false, false, 0,   0,    0  },
  { false, true,  500, 1000, 12 },
  { false, false, 0,   0,    0  },
  { false, true,  500, 1000, 8  },
  { false, false, 0,   0,    0  },
  { false, true,  500, 1000, 4  },
  { false, false, 0,   0,    0  },
  { false, true,  500, 1000, 0  },
  { false, false, 0,   0,    0  },
}};



/*
 * parse_line
 *
 * Read and parse a single line, then update edge and pulse lists.
 *
 *  edge_list
 *    Queue of incoming edge transitions.
 *  pulse_list
 *    Queue of incoming pulses.
 *  in
 *    Input stream to read from.
 */
void
parse_line(std::deque<Edge>& edge_list,
           std::deque<Pulse>& pulse_list,
           std::istream& in) {
  // Scratch area for incoming line.
  static std::array<char, 128> line_buf;
  line_buf.fill(0);

  if (in) {
    in.getline(&line_buf[0], 128);
    LOG << "Read: " << &line_buf[0] << std::endl;

    // Edge parameters to be parsed from input line.
    int event_num = 0;
    int state = -1;
    uint64_t mtime = 0;
    uint64_t ktime = 0;

    // Match expected input pattern.
    int ret = std::sscanf(&line_buf[0],
                "Extsync (%d): GPIO: %d mtime:%" SCNu64 " ktime:%" SCNu64 "\n",
                &event_num, &state, &mtime, &ktime);

    // Succesful match.
    if (4 == ret) {
      // Need at least one preceding edge in order to generate a pulse.
      if (!edge_list.empty()) {
        Edge& left = edge_list.back();
        pulse_list.push_back(
            Pulse {
              /* .high =            */ (left.state == 1),
              /* .duration_us =     */ (ktime - left.ktime),
              /* .event_num_left =  */ left.event_num,
              /* .event_num_right = */ event_num
            }
        );
        LOG << "Parse: Pulse { " << (left.state == 1) << ", " << (ktime - left.ktime)
            << ", " << left.event_num << ", " << event_num << " }"
            << std::endl;
      }
      edge_list.push_back(
          Edge {
            /* .ktime =     */ ktime,
            /* .mtime =     */ mtime,
            /* .state =     */ state,
            /* .event_num = */ event_num
          }
      );
      LOG << "Parse: Edge { " << ktime << ", " << mtime << ", " << state
          << ", " << event_num << " }" << std::endl;
    } else {
      LOG << "Parse error (std::sscanf returned: " << ret << ")" << std::endl;
    }

  } else {
    LOG << "Can't read." << std::endl;
  }
  
}


/*
 * match_and_decode
 *
 * Attempt to find a matching payload pattern at front of incoming stream.
 * Attempt to decode pattern.
 *
 *  return:
 *    If pattern found, create and return a Trigger containing the matched
 *    trigger timestamps, payload data, and error. Otherwise, return none.
 *
 *  edge_list
 *    Queue of incoming edge transitions.
 *  pulse_list
 *    Queue of incoming pulses.
 *  filter
 *    Template for matched filter.
 *  decoder
 *    Template for assigning trigger edge timestamp and for decoding payload.
 */
boost::optional<Trigger>
match_and_decode(std::deque<Edge>& edge_list,
                 std::deque<Pulse>& pulse_list,
                 std::vector<Pattern>& filter,
                 std::vector<Codeword>& decoder) {

  const double total_weights = std::accumulate(filter.begin(), filter.end(), 0,
                                      [](double sum, Pattern& next) {
                                        return sum + next.weight;
                                      });

  // Input stream needs at least as many pulses as filter size in order to
  // attempt a match.
  if (pulse_list.size() < filter.size()) {
    return boost::none;
  }

  Trigger t {
      /* .ktime =           */ 0,
      /* .mtime =           */ 0,
      /* .payload_present = */ false,
      /* .frame_num =       */ 0,
      /* .mse =             */ 0.0
  };

  for (size_t j = 0; j < filter.size(); ++j) {
    Pulse& pulse = pulse_list[j];
    Edge& edge = edge_list[j];

    // Bailout condition -- these are always required for a successful match.
    if ((pulse.high != filter[j].high) || 
        (pulse.event_num_left + 1 != pulse.event_num_right)
    ) {
      return boost::none;
    }

    // This pulse will match. Compute match error.
    uint64_t timediff_us = 0;
    if (pulse.duration_us < filter[j].min_duration_us) {
      timediff_us = filter[j].min_duration_us - pulse.duration_us;
    } else if (pulse.duration_us > filter[j].max_duration_us) {
      timediff_us = pulse.duration_us - filter[j].max_duration_us;
    }
    double error = filter[j].weight * (static_cast<double>(timediff_us * timediff_us) / 1000000.0);
    t.mse += error;

    // Mark the trigger timestamp.
    if (decoder[j].trigger) {
      t.ktime = edge.ktime;
      t.mtime = edge.mtime;
    }

    // Decode payload nibbles.
    if (decoder[j].nibble) {
      int64_t n = ((pulse.duration_us - decoder[j].subtract) / decoder[j].divide);
      // Clamp and warn.
      if (n < 0) {
        LOG << "Decode error (n = " << n << ")" << std::endl;
        n = 0;
      } else if (n >= 16) {
        LOG << "Decode error (n = " << n << ")" << std::endl;
        n = 15;
      }
      LOG << "  n=" << n << std::endl;
      t.frame_num |= (static_cast<uint32_t>(n) << decoder[j].leftshift);
    }
  }

  // Successfully matched and decoded.
  t.payload_present = true;
  t.mse /= total_weights;
  LOG << "Matched: Trigger { " << t.ktime << ", " << t.mtime << ", "
      << t.payload_present << ", " << t.frame_num << ", " << t.mse << " }"
      << std::endl;
  return t;
}



void
advance_stream(std::deque<Edge>& edge_list,
               std::deque<Pulse>& pulse_list,
               std::vector<Pattern>& filter) {
  if (pulse_list.size() > filter.size()) {
    pulse_list.pop_front();
    edge_list.pop_front();
  }
}

bool done(std::deque<Pulse>& pulse_list,
          std::vector<Pattern>& filter,
          std::istream& in) {
  return (!in && pulse_list.size() <= filter.size());
}





int main(int argc, char *argv[]) {

  // Parse command line args.
  if (argc > 1) {
    for (int i = 1; i < argc; ++i) {
      if (0 == std::strcmp(argv[i], "--debug") || 0 == std::strcmp(argv[i], "-d")) {
        debug_log = true;
      }
    }
  }

  // Queue of incoming edge transitions to be processed.
  std::deque<Edge>  edge_list;

  // Queue of pulses to be processed
  std::deque<Pulse> pulse_list;

  // Decoded trigger result
  boost::optional<Trigger> trig;


  // Run until no more input, no more matches, and we can't advance stream.
  while(!done(pulse_list, payload_filter, std::cin)) {
    parse_line(edge_list, pulse_list, std::cin);
    trig = match_and_decode(edge_list, pulse_list, payload_filter, payload_decoder);

    if (trig && trig->mse < 1000.0) {
      // Old format:
      //std::cout << "Trigger { " << trig->ktime << ", " << trig->mtime << ", "
      //          << trig->payload_present << ", " << trig->frame_num << ", "
      //          << trig->mse << " }" << std::endl;

      // New format:
      std::cout << "{ \"ktime\": " << trig->ktime
                << ", \"mtime\": " << trig->mtime
                << ", \"payload_present\": " << std::boolalpha << trig->payload_present
                << ", \"frame_number\": " << trig->frame_num
                << ", \"mean_squared_error\": " << trig->mse
                << " }" << std::endl;
    }
    
    advance_stream(edge_list, pulse_list, payload_filter);
  }


  return 0;
}


