#ifndef CONTROLLER_HH
#define CONTROLLER_HH

#include <cstdint>
#include <vector> 

/* Congestion controller interface */

class Controller
{
private:

  enum bbr_state {STARTUP, DRAIN, PROBE_BW, PROBE_RTT};

  /* Stores data point of network behavior at a specific time.
   * EG, rtt of packet or delivery rate. */
  typedef struct sample {
    double data_point;
    uint64_t time_seen;

    sample(double data_point, uint64_t time_seen): data_point(data_point), time_seen(time_seen) {}

    bool operator <(const sample& rhs) {
      return time_seen < rhs.time_seen;
    }
  } sample;

  bool debug_; /* Enables debugging output */

  /* Current state in the BBR FSM */
  bbr_state state;

  float a;
  unsigned int super_congested;
  unsigned int num_congested;
  unsigned int num_acks;
  /* Max time (in milliseconds) an rt sample is valid.
   * Time windo for RTProp calculation*/
  unsigned int rt_sample_timeout; 
  /* All observed RTTs within time window rt_sample_timeout*/
  std::vector<sample> rt_filter; 
  /* Current propagation delay (RTprop) estimate */
  double rt_estimate; 


  uint64_t rt_estimate_last_updated; /* Time (in milliseconds) since estimate was updated */
  unsigned int stale_update_threshold; /* Max time (in milliseconds) an rt estimate can remain the same before we probe for a new estimate */

  /* Max time (in milliseconds) a delivery rate sample is valid */
  unsigned int btlbw_sample_timeout(); 
  /* Observed delivery rates within time window btl_bw_sample_timeout() */
  std::vector<sample> btlbw_filter; 
  /* Current bottleneck bandwidth estimate >= delivery rate */
  double btlbw_estimate; 
  

  unsigned int startup_rounds_without_increase;

  int cwnd;

  unsigned int num_packets_delivered;

  /* Number of inflight packets */
  unsigned int inflight;

  uint64_t delivered;

  uint64_t delivered_time;

  /* Multiplier of bdp to calculate allowed # inflight packets */
  double cwnd_gain;

  double pacing_gain;

  uint64_t next_send_time;

  /* Removes samples that have timed out from a filter */
  static void remove_old_samples(std::vector<sample>& filter, uint64_t time_now, uint64_t timeout); 

public:
  /* Public interface for the congestion controller */

  /* Default constructor */
  Controller( const bool debug );

  /* Get current window size, in datagrams */
  unsigned int window_size();

  /* A datagram was sent */
  void datagram_was_sent( const uint64_t sequence_number,
			  const uint64_t send_timestamp,
        const uint64_t payload_length,
			  const bool after_timeout );

  /* An ack was received */
  void ack_received( const uint64_t sequence_number_acked,
		     const uint64_t send_timestamp_acked,
		     const uint64_t recv_timestamp_acked,
		     const uint64_t timestamp_ack_received,
         const uint64_t payload_length,
         const uint64_t packet_delivered,
         const uint64_t packet_delivered_time);

  /* How long to wait (in milliseconds) if there are no acks
     before sending one more datagram */
  unsigned int timeout_ms();

  /* Returns true if inflight packets is < cwnd_gain * bdp */
  bool window_is_open();

  /* Returns true if time is >= next send time */
  bool should_send_packet();

  uint64_t get_delivered();

  uint64_t get_delivered_time();
};

#endif
