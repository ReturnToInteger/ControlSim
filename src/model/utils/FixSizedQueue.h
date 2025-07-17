#pragma once  
#include <deque>  
#include <stdexcept>
#include <vector>


namespace model {
	template<typename T>
	class FixSizedQueue {
	public:
		FixSizedQueue(int size) : m_q({}), m_size(size) 
		{
			if (size < 1) {
				throw std::out_of_range("Size has to be at least 1.");
			}
		}
		void push(T const& item) 
		{
			while (m_q.size() > m_size - 1) {
				pop();
			}
			m_q.push_back(item);
		}
		bool pop() 
		{
			if (m_q.empty()) return false;
			m_q.pop_front();
			return true;
		}
		// Return default value in case it's empty, quick fix for Vehicle for Path Planning input
		T back() const 
		{
			if (!m_q.empty())
				return m_q.back();
			else
				return T();
		}
		T front() const
		{
			if (!m_q.empty())
				return m_q.front();
			else return T();
		}
		explicit operator std::vector<T>() const
		{
			return std::vector<T>(m_q.begin(), m_q.end());
		}
		size_t size() { return m_q.size(); }
		auto begin() const { return m_q.begin(); }
		auto end() const { return m_q.end(); }
		std::deque<T> const& get() const { return m_q; }
		void set(std::deque<T> const& q) { m_q = q; };
	private:
		std::deque<T> m_q;
		const int m_size;
	};
}
