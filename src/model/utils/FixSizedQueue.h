#pragma once  
#include <deque>  
#include <stdexcept>


namespace model {
	template<typename T>
	class FixSizedQueue {
	public:
		FixSizedQueue(int size) : _q({}), _size(size) {
			if (size < 1) {
				throw std::out_of_range("Size has to be at least 1.");
			}
		}
		void push(T const& item) {
			while (_q.size() > _size - 1) {
				pop();
			}
			_q.push_back(item);
		}
		bool pop() {
			if (_q.empty()) return false;
			_q.pop_front();
			return true;
		}
		std::deque<T> const& get() const { return _q; }
	private:
		std::deque<T> _q;
		const int _size;
	};
}
