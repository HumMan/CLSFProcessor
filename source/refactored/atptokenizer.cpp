#include "atptokenizer.h"

using namespace CLSFProcessor;

class TATPTokenizer
{
	//Source : (Token ('/' Parameters | '\n'))* '\0';
	//Parameters : Token ((',' Token)+ | '\n');
	//Token : '[^/,\n\0]';

private:
	const char *c;
	char token[1024];
	int line, col;
	void NextChar()
	{
		c++;
		col++;
	}
	void GetToken()
	{
		const char *start = c;
		while (*c != 10 && *c != 13 && *c != 0 && *c != ','&&*c != '/')
			NextChar();
		if ((c - start) >= sizeof(token))
		{
			char buf[255];
			//TODO
			//std::string s = (boost::format("(line %1% col %2%) Token is too long!(max length = %3%)") % line%col % (sizeof(token) - 1)).str();
			//sprintf(buf, "(line %i col %i) Token is too long!(max length = %i)", line, col, sizeof(token) - 1);
			throw std::string(buf);
		}
		memcpy(token, start, c - start);
		token[c - start] = 0;
	}
	void ParseParams(TCLSFToken& t, const char* &start)
	{
		int i = 0;
		do
		{
			GetToken();
			t.AddParam(token);
			if (*c == 13 || *c == 0 || *c == 10)break;
			else
			{
				assert(*c == ',' || *c == '/');
				NextChar();
			}
		} while (true);
		if (*c == 13 || *c == 10)
		{
			NextChar();
			if (*c == 10)NextChar();
		}
	}
public:
	TATPTokenizer()
	{
	}
	std::vector<TCLSFToken> Parse(const char* use_source)
	{
		std::vector<TCLSFToken> tokens;
		tokens.reserve(1000);

		c = use_source;
		col = 1;
		line = 1;
		while (*c != '\0')
		{
			if (*c == '$')
			{
				while (*c != 13 && *c != 10 && *c != 0)NextChar();
				if (*c == 0)break;
			}
			else if (*c == 13 || *c == 10)
			{
				NextChar();
				if (*c == 10)NextChar();
				line++; col = 1;
			}
			else
			{
				const char* start = c;
				GetToken();
				TCLSFToken t;
				t.SetName(token);
				if (*c == '/')
				{
					NextChar();
					ParseParams(t, start);
				}
				else if (*c == ',')
				{
					char buf[2550];
					sprintf(buf, "(line %i col %i) Error in command, '/' or newline or end of file expected", line, col, sizeof(token) - 1);
					//printf(buf);
					throw std::string(buf);
				}
				tokens.push_back(t);
			}
		}

		return tokens;
	}
};

std::vector<TCLSFToken> CLSFProcessor::Parse(const char* use_source)
{
	TATPTokenizer tokenizer;
	return tokenizer.Parse(use_source);
}