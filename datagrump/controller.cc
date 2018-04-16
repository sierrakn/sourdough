#include <iostream>

#include "controller.hh"
#include "timestamp.hh"
#include <algorithm>
#include "math.h"

using namespace std;

/* Default constructor */
Controller::Controller( const bool debug )
  : debug_( debug ), state(STARTUP), rt_sample_timeout(10000), 
      rt_filter(), rt_estimate(0),
      rt_estimate_last_updated(0), stale_update_threshold(10000),
      btlbw_filter(), btlbw_estimate(0), startup_rounds_without_increase(0),
      cwnd(1), inflight(0), cwnd_gain(2 / log(2)), pacing_gain(2 / log(2)),
      next_send_time(0)
{}

/* Get current window size, in datagrams */
unsigned int Controller::window_size()
{  
  if ( debug_ ) {
    cerr << "At time " << timestamp_ms()
	 << " window size is " << cwnd << endl;
  }

  return cwnd;
}

/* A datagram was sent */
void Controller::datagram_was_sent( const uint64_t sequence_number,
				    /* of the sent datagram */
				    const uint64_t send_timestamp,
            /* in milliseconds */
            const uint64_t payload_length,
            /* in bytes */
				    const bool after_timeout
				    /* datagram was sent because of a timeout */ )
{
  if ( debug_ ) {
    cerr << "At time " << send_timestamp
	 << " sent datagram " << sequence_number << " (timeout = "  << after_timeout << ")\n";
  }

  // cerr << "payload_length = " << payload_length << " btlbw_estimate= " << btlbw_estimate << endl;
  inflight++;
  next_send_time = send_timestamp + (payload_length / (pacing_gain * btlbw_estimate));
  if (debug_) {
    cerr << "pacing_gain = " << pacing_gain << "time " << (payload_length / (pacing_gain * btlbw_estimate)) << endl;
  }
}

/* An ack was received 
 * In BBR: Each ack provides new RTT and average delivery rate measurements 
 * that update the RTprop and BtlBw estimates.
 */
void Controller::ack_received( const uint64_t sequence_number_acked,
			       /* what sequence number was acknowledged */
			       const uint64_t send_timestamp_acked,
			       /* when the acknowledged datagram was sent (sender's clock) */
			       const uint64_t recv_timestamp_acked,
			       /* when the acknowledged datagram was received (receiver's clock)*/
			       const uint64_t timestamp_ack_received, 
             /* when the ack was received (by sender) */
             const uint64_t payload_length)
             /* payload length of message acked */
{
  if ( debug_ ) {
    cerr << "At time " << timestamp_ack_received
	 << " received ack for datagram " << sequence_number_acked
	 << " (send @ time " << send_timestamp_acked
	 << ", received @ time " << recv_timestamp_acked << " by receiver's clock)"
   << ", payload length = " << payload_length
	 << endl;
  }

  inflight--;
  double rtt = timestamp_ack_received - send_timestamp_acked;

  // Calculate new RTprop estimate (min RTT over time window rt_sample_timeout)
  rt_filter.emplace_back(rtt, timestamp_ack_received);
  remove_old_samples(rt_filter, timestamp_ack_received, rt_sample_timeout);
  sample min_rtt_sample = *std::min_element(rt_filter.begin(), rt_filter.end());
  if (min_rtt_sample.data_point != rt_estimate) {
    rt_estimate = min_rtt_sample.data_point;
    rt_estimate_last_updated = timestamp_ack_received;
  }
  // TODO deal with no update in 10 seconds, initial estimates

  // Calculate new BtlBw estimate
  double delivery_rate = payload_length / rtt;
  btlbw_filter.emplace_back(delivery_rate, timestamp_ack_received);
  remove_old_samples(btlbw_filter, timestamp_ack_received, btlbw_sample_timeout());
  cerr << "btlbw " << btlbw_filter.size() << endl;
  sample max_btlbw_sample = *std::max_element(btlbw_filter.begin(), btlbw_filter.end());
  btlbw_estimate = max_btlbw_sample.data_point;

  cerr << "rt = " << rt_estimate << ", btlbw = " << btlbw_estimate << endl;
}

/* How long to wait (in milliseconds) if there are no acks
   before sending one more datagram */
unsigned int Controller::timeout_ms()
{
  return 1000; /* timeout of one second */
}

bool Controller::window_is_open()
{
  const uint64_t bdp = rt_estimate * btlbw_estimate;
  unsigned int cwnd = bdp * cwnd_gain;
  if (cwnd == 0) {
    cwnd = 1; 
  }
  
  cerr << "inflight = " << inflight << "cwnd = " << cwnd << endl;
  if (inflight >= cwnd) {
    return false;
  }
  
  if (timestamp_ms() < next_send_time) {
    return false;
  }
  return true;
}


/* Max time (in milliseconds) a delivery rate sample is valid */
unsigned int Controller::btlbw_sample_timeout() {
  unsigned int num_rtts = 8;
  return num_rtts * rt_estimate;
}

/* Removes sample data points that have timed out from a filter */
void Controller::remove_old_samples(std::vector<sample>& filter, uint64_t time_now, unsigned int timeout) {
  std::remove_if(filter.begin(), filter.end(), 
   [time_now, timeout](const sample& s) { 
    return time_now - s.time_seen > timeout; 
  });
}
