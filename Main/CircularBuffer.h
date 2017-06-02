#ifndef _CIRCULAR_BUFFER_H_
#define _CIRCULAR_BUFFER_H_

template <typename T, unsigned CAPACITY>
class CircularBuffer
{
public:
	unsigned GetCapacity() { return CAPACITY; }

	unsigned GetSize() { return (CAPACITY + m_BackId - m_FrontId) % CAPACITY; }

	void PushFront(const T &_element)
	{
		Assert(!IsFull());
		m_FrontId = (m_FrontId + CAPACITY - 1) % CAPACITY;
		m_Buffer[m_FrontId] = _element;
	}

	void PushBack(const T &_element)
	{
		Assert(!IsFull());
		m_Buffer[m_BackId] = _element;
		m_BackId = (m_BackId + 1) % CAPACITY;
	}

	void PopFront()
	{
		Assert(m_FrontId != m_BackId);
		m_FrontId = (m_FrontId + 1) % CAPACITY;
	}

	void PopBack()
	{
		Assert(m_FrontId != m_BackId);
		m_BackId = (m_BackId + CAPACITY - 1) % CAPACITY;
	}

	void Clear() { m_BackId = m_FrontId = 0; }
	bool IsEmpty() { return m_FrontId == m_BackId; }
	bool IsFull() { return (((m_BackId + 1) % CAPACITY) == m_FrontId); }

	const T& operator[](unsigned i) const { return m_Buffer[i]; }
	T& operator[](unsigned i) { return m_Buffer[i]; }

	T& Front()
	{
		Assert(m_FrontId != m_BackId);
		return m_Buffer[m_FrontId];
	}
	const T& Front() const
	{
		Assert(m_FrontId != m_BackId);
		return m_Buffer[m_FrontId];
	}

	T& Back()
	{
		Assert(m_FrontId != m_BackId);
		unsigned id = (m_BackId + CAPACITY - 1) % CAPACITY;
		return m_Buffer[id];
	}
	const T& Back() const
	{
		Assert(m_FrontId != m_BackId);
		unsigned id = (m_BackId + CAPACITY - 1) % CAPACITY;
		return m_Buffer[id];
	}

	template <typename Function>
	void Apply(Function fct)
	{
		for (unsigned i = m_FrontId; i != m_BackId; )
		{
			fct(m_Buffer[i]);
			i++;
			if (i == CAPACITY)
				i = 0;
		}
	}

private:
	T m_Buffer[CAPACITY];
	unsigned m_FrontId = 0;
	unsigned m_BackId = 0;
};

#endif
