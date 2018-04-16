#ifndef CONTROLLER_HH
#define CONTROLLER_HH

#include <cstdint>
#include <vector> 

/* Congestion controller interface */

class Controller
{
private:

  /* Encapsulates an observation about network behavior */
  typedef struct sample {
    float data_point;
    uint64_t time_seen;

    sample(float data_point, uint64_t time_seen): data_point(data_point), time_seen(time_seen) {}

    bool operator <(const sample& rhs) {
      return time_seen < rhs.time_seen;
    }
  } sample;

  bool debug_; /* Enables debugging output */

  std::vector<sample> rt_filter; /* Tracks observed RTTs */
  float rt_estimate; /* Current propagation delay estimate */
  unsigned int rt_sample_timeout; /* Max time (in milliseconds) an rt sample is valid */

  uint64_t rt_estimate_last_updated; /* Time (in milliseconds) since estimate was updated */
  unsigned int stale_update_threshold; /* Max time (in milliseconds) an rt estimate can remain the same before we probe for a new estimate */

  std::vector<sample> btl_bw_filter; /* Tracks observed delivery rates */
  float btl_bw_estimate; /* Current bottleneck bandwidth estimate */
  unsigned int btl_bw_sample_timeout(); /* Max time (in milliseconds) a delivery rate sample is valid */

  float cwnd_gain;
  float pacing_gain;

  unsigned int startup_rounds_without_increase;

  static void remove_old_samples(uint64_t time_now, unsigned int timeout, std::vector<sample>& filter); /* Removes samples that have timed out from a filter */

public:
  /* Public interface for the congestion controller */

  /* Default constructor */
  Controller( const bool debug );

  /* Get current window size, in datagrams */
  unsigned int window_size();

  /* A datagram was sent */
  void datagram_was_sent( const uint64_t sequence_number,
			  const uint64_t send_timestamp,
			  const bool after_timeout );

  /* An ack was received */
  void ack_received( const uint64_t sequence_number_acked,
		     const uint64_t send_timestamp_acked,
		     const uint64_t recv_timestamp_acked,
		     const uint64_t timestamp_ack_received );

  /* How long to wait (in milliseconds) if there are no acks
     before sending one more datagram */
  unsigned int timeout_ms();
};

#endif
