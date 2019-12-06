#ifndef MAPLAB_MESSAGE_SYNC_H_
#define MAPLAB_MESSAGE_SYNC_H_

#include <map>
#include <list>
#include <mutex>
#include <functional>
#include <algorithm>

#include <glog/logging.h>

namespace maplab {

template<typename T>
struct ROSTimestampExtractor {
  int64_t operator()(const T& data) {
    return data->header.stamp.toNSec();
  }
};

template <typename FlowA, typename FlowB,
          typename TimestampExtractorA = ROSTimestampExtractor<FlowA>,
          typename TimestampExtractorB = ROSTimestampExtractor<FlowB>>
class MessageSync {
  public:
     typedef std::function<void(const FlowA&, const FlowB&)>
      SynchronizedDataCallback;

    explicit MessageSync(const double ts_epsilon_ns = 2.4e7) 
      : ts_epsilon_ns_(ts_epsilon_ns), 
        last_timestamp_a_(std::numeric_limits<int64_t>::min()),
        last_timestamp_b_(std::numeric_limits<int64_t>::min()) {
    }

    void callback1(const FlowA &flow) {
      {
        std::lock_guard<std::mutex> lock(m_list_a_);
        const int64_t timestamp_a = TimestampExtractorA()(flow);
        CHECK_GT(timestamp_a, last_timestamp_a_) << "Data from flow A not in temporal order.";
        list_a_.emplace_back(flow);
        last_timestamp_a_ = timestamp_a;
      }
      findAndProcessMatches();
    }
    void callback2(const FlowB &flow) {
      {
        std::lock_guard<std::mutex> lock(m_map_b_);
        const int64_t timestamp_b = TimestampExtractorB()(flow);
        CHECK_GT(timestamp_b, last_timestamp_b_) << "Data from flow B not in temporal order.";
        CHECK(map_b_.emplace(timestamp_b, flow).second);
        last_timestamp_b_ = timestamp_b;
      }
      findAndProcessMatches();
    }

    void clear() {
      std::lock_guard<std::mutex> lock_a(m_list_a_);
      std::lock_guard<std::mutex> lock_b(m_map_b_);
      list_a_.clear();
      map_b_.clear();
    }

    void registerCallback(const SynchronizedDataCallback& cb) {
      CHECK(cb);
      std::lock_guard<std::mutex> lock(m_sync_data_callback_);
      sync_data_callback_ = cb;
    }

  private:
    void findAndProcessMatches() {
      std::lock_guard<std::mutex> lock_a(m_list_a_);
      std::lock_guard<std::mutex> lock_b(m_map_b_);

      if (list_a_.empty() || map_b_.empty()) return;
      deleteOrphans();
      findMatch();
    }

    void deleteOrphans() {
      // Remove all elements in map B up to (but no including) the lowest
      // timestamp in A; they will never match assuming temporal order of input.
      const int64_t lowest_timestamp_a = TimestampExtractorA()(list_a_.front());
      auto it_b = map_b_.lower_bound(lowest_timestamp_a);
      map_b_.erase(map_b_.begin(), it_b);
    }

    void findMatch() {
      // Go over all elements of the list A and try to find matching elements in
      // map B. We also remove all entries that won't match in the future
      // (assuming input is in temporal order).
      //  list_a:  front -> oldest/smallest - back   -> newest/largest timestamp
      //  map_a:   begin -> oldest/smallest - rbegin -> newest/largest timestamp
      auto it_a = list_a_.begin();
      auto it_a_end = list_a_.end();
      while (it_a != list_a_.end()) {
        const int64_t timestamp_a = TimestampExtractorA()(*it_a);

        // Early exit if no matches are possible.
        if (map_b_.empty()) {
          break;
        }

        // Nothing to match if current timestamp in A is ahead of the most
        // recent element in map B.
        const int64_t largest_timestamp_b = map_b_.rbegin()->first;
        if (timestamp_a > largest_timestamp_b) {
          break;
        }

        // Try to find a matching element in map B.
        //auto it_b = map_b_.find(timestamp_a);
        auto it_b = std::find_if(map_b_.begin(), map_b_.end(),
            [&] (const std::pair<uint64_t, FlowB> &flow) {
              if (timestamp_a > flow.first) {
                return timestamp_a - flow.first <= ts_epsilon_ns_;
              }
              else {
                //std::cout << "diff: " << flow.first - timestamp_a << std::endl;
                return flow.first - timestamp_a <= ts_epsilon_ns_;
              }
            });
        if (it_b != map_b_.end()) {
          // Process the match and remove the element from list A. We will remove
          // all elements from map B at the end.
          {
            std::lock_guard<std::mutex> lock(m_sync_data_callback_);
            if (sync_data_callback_) {
              sync_data_callback_(*it_a, it_b->second);
            }
          }

          map_b_.erase(map_b_.begin(), ++it_b);
          it_a = list_a_.erase(it_a);
          continue;
        }  
        ++it_a;
      }
    }


    const uint64_t ts_epsilon_ns_;
    int64_t last_timestamp_a_;
    std::list<FlowA> list_a_;
    mutable std::mutex m_list_a_;

    int64_t last_timestamp_b_;
    std::map<int64_t, FlowB> map_b_;
    mutable std::mutex m_map_b_;

    SynchronizedDataCallback sync_data_callback_;
    std::mutex m_sync_data_callback_;
};

}  // namespace maplab

#endif // MAPLAB_MESSAGE_SYNC_H_