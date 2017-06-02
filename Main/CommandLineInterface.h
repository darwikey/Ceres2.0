#ifndef CLI_H
#define CLI_H


#define CLI_DEL_CHAR  127
#define CLI_ESC_CHAR  27
#define CLI_HISTORY_CHAR 178
#define CLI_DELIMITER ' '
#define CLI_BUFFER_SIZE 128
#define CLI_HISTORY_MAX 8

#define CLI_ARG_LENGTH 20
#define CLI_MAX_ARG 5

template <unsigned CAPACITY>
class CString
{
public:
	CString() { m_Buffer[0] = '\0'; }
	CString(const char* s) { strcpy(m_Buffer, s); }
	CString(const CString &s) { strcpy(m_Buffer, s.m_Buffer); }
	unsigned Length() { return strlen(m_Buffer); }
	char& operator[](unsigned i) { return m_Buffer[i]; }
	//const char& operator[] const (unsigned i) { return m_Buffer[i]; }
	const char* Str() const { return m_Buffer; }
	char* Str() { return m_Buffer; }
	void operator=(const CString &s) { strcpy(m_Buffer, s.m_Buffer); }

private:
	char m_Buffer[CAPACITY];
};

class CommandLineInterface
{
public:
	static CommandLineInterface Instance;
	void Init();
	void Task();
	void PrintHelp();

private:
	struct Command
	{
		const char *cmd;
		const char *help;
		typedef void (*FunctionDecl)(const char [CLI_MAX_ARG][CLI_ARG_LENGTH], int);
		FunctionDecl function;
	};

	Command m_Command[128];

	char m_Buffer[CLI_BUFFER_SIZE];
	CircularBuffer<CString<CLI_BUFFER_SIZE>, CLI_HISTORY_MAX> m_History;
	int m_CurHistoryId = 0;
	int m_LastHistoryId = 0;
	int m_CurPos = 0;
};
#endif /* CLI_H */
