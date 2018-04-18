#include <iostream>

#include "controller.hh"
#include "timestamp.hh"
#include <algorithm>
#include "math.h"

using namespace std;

/* Default constructor */
Controller::Controller( const bool debug )
  : debug_( debug ), a(2), num_congested(0), num_acks(0),
    rt_sample_timeout(10000), rt_filter(), rt_estimate(0), cwnd(1)
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
            const bool after_timeout
            /* datagram was sent because of a timeout */ )
{
  if ( debug_ ) {
    cerr << "At time " << send_timestamp
   << " sent datagram " << sequence_number << " (timeout = "  << after_timeout << ")\n";
  }

  float b = 0.6;
  if (after_timeout) {
    // cerr << "TIMEOUT" << endl << endl << endl << endl << endl << endl << endl << endl;
    cwnd = cwnd * b;
  }
}

/* An ack was received */
void Controller::ack_received( const uint64_t sequence_number_acked,
             /* what sequence number was acknowledged */
             const uint64_t send_timestamp_acked,
             /* when the acknowledged datagram was sent (sender's clock) */
             const uint64_t recv_timestamp_acked,
             /* when the acknowledged datagram was received (receiver's clock)*/
             const uint64_t timestamp_ack_received 
             /* when the ack was received (by sender) */ )
{
  if ( debug_ ) {
    cerr << "At time " << timestamp_ack_received
   << " received ack for datagram " << sequence_number_acked
   << " (send @ time " << send_timestamp_acked
   << ", received @ time " << recv_timestamp_acked << " by receiver's clock)"
   << endl;
  }

  double rtt = timestamp_ack_received - send_timestamp_acked;

  // Calculate new RTprop estimate (min RTT over time window rt_sample_timeout)
  rt_filter.emplace_back(rtt, timestamp_ack_received);
  remove_old_samples(rt_filter, timestamp_ack_received, rt_sample_timeout);
  sample min_rtt_sample = *std::min_element(rt_filter.begin(), rt_filter.end());
  rt_estimate = min_rtt_sample.data_point;

  // cerr << "rt = " << rt_estimate << ", a = " << a << endl;

  double A_MIN = 0.5;
  double A_MAX = 3.0;

  if (rtt > 80) {
    num_congested++;
    a = a - 0.3;
    if (a < A_MIN) {
      a = A_MIN;
    }
    if (num_congested % 3 == 1) {
      cwnd--;
    }
  } else {
    num_congested = 0;
  }

  if (cwnd <= 0) {
    cwnd = 1;
  }

  unsigned int required_acks = cwnd/a;
  num_acks++;
  if (num_acks >= required_acks) {
    num_acks -= required_acks;
    cwnd += 1; 
    a = a + 0.15;
    if (a > A_MAX) {
      a = A_MAX;
    }
  }
}

/* How long to wait (in milliseconds) if there are no acks
   before sending one more datagram */
unsigned int Controller::timeout_ms()
{
  if (rt_estimate < 50) {
    return 50;
  }
  if (rt_estimate > 80) {
    return 80;
  }
  return rt_estimate;
}

/* Removes sample data points that have timed out from a filter */
void Controller::remove_old_samples(std::vector<sample>& filter, uint64_t time_now, uint64_t timeout) {
  filter.erase(std::remove_if(filter.begin(), filter.end(), 
   [time_now, timeout](const sample& s) { 
    return (time_now - s.time_seen) > timeout; 
  }), filter.end());
}
