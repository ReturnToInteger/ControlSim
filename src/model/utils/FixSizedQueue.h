#pragma once  
#include <deque>  
#include <stdexcept>
#include <vector>


namespace model {
	template<typename T>
	class FixSizedQueue {
	public:
		FixSizedQueue(int size) : _q({}), _size(size) 
		{
			if (size < 1) {
				throw std::out_of_range("Size has to be at least 1.");
			}
		}
		void push(T const& item) 
		{
			while (_q.size() > _size - 1) {
				pop();
			}
			_q.push_back(item);
		}
		bool pop() 
		{
			if (_q.empty()) return false;
			_q.pop_front();
			return true;
		}
		// Return default value in case it's empty, quick fix for Vehicle for Path Planning input
		T back() const 
		{
			if (!_q.empty())
				return _q.back();
			else
				return T();
		}
		T front() const
		{
			if (!_q.empty())
				return _q.front();
			else return T();
		}
		explicit operator std::vector<T>() const
		{
			return std::vector<T>(_q.begin(), _q.end());
		}
		size_t size() { return _q.size(); }
		auto begin() const { return _q.begin(); }
		auto end() const { return _q.end(); }
		std::deque<T> const& get() const { return _q; }
		void set(std::deque<T> const& q) { _q = q; };
	private:
		std::deque<T> _q;
		const int _size;
	};
}
