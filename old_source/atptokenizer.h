#pragma once

#include <boost/static_assert.hpp>
#include <boost/format.hpp>

#include <string.h>
#include <baluLib.h>

using namespace std;

class TATPTokenizer
{
	//Source : (Token ('/' Parameters | '\n'))* '\0';
	//Parameters : Token ((',' Token)+ | '\n');
	//Token : '[^/,\n\0]';
public:
	struct TATPToken
	{
		string* name;
		int params_count;
		std::string** params;
		std::string& operator[](int i)
		{
			assert(i>=0&&i<params_count);
			return *(params[i]);
		}
	};
private:
	TAllocator<std::string,255> names;
	TVector<TATPToken> tokens;
	char *c;
	char token[1024];
	int line,col;
	void NextChar()
	{
		c++;
		col++;
	}
	void GetToken()
	{
		char *start=c;
		while(*c!=10&&*c!=13&&*c!=0&&*c!=','&&*c!='/')
			NextChar();
		if((c-start)>=sizeof(token))
		{
			char buf[255];
			string s=(boost::format("(line %1% col %2%) Token is too long!(max length = %3%)")%line%col%(sizeof(token)-1)).str();
			sprintf(buf,"(line %i col %i) Token is too long!(max length = %i)",line,col,sizeof(token)-1);
			throw string(buf);
		}
		memcpy(token,start,c-start);
		token[c-start]=0;
	}
	void ParseParams(TATPToken& t,char* &start,bool only_count)
	{
		int i=0;
		do
		{
			GetToken();
			if(!only_count)
			{
				std::string* temp=names.New();
				temp->append(token);
				t.params[i++]=temp;
			}else
				t.params_count++;
			if(*c==13||*c==0||*c==10)break;
			else
			{
				assert(*c==','||*c=='/');
				NextChar();
			}
		}while(true);
		if(*c==13||*c==10)
		{
			NextChar();
			if(*c==10)NextChar();
		}
	}
public:
	std::string* NewName(){
		return names.New();
	}
	TATPToken operator[](int i)
	{
		return tokens[i];
	}
	int GetTokensCount(){
		return tokens.GetCount();
	}
	/*TATPTokenizer(TVector<TATPToken> use_tokens)
	{
		tokens=use_tokens;
	}*/
	void SetTokens(TVector<TATPToken> use_tokens)
	{
		tokens=use_tokens;
	}
	TATPTokenizer()
	{
	}
	TATPTokenizer(char* const use_source)
	{
		tokens.SetReserve(1000);
		c=use_source;
		col=1;
		line=1;
		while(*c!='\0')
		{
			if(*c=='$')
			{
				while(*c!=13&&*c!=10&&*c!=0)NextChar();
				if(*c==0)break;
			}
			else if(*c==13||*c==10)
			{
				NextChar();
				if(*c==10)NextChar();
				line++;col=1;
			}
			else
			{
				char* start=c;
				GetToken();
				TATPToken t;
				t.name=names.New();
				t.name->append(token);
				t.params_count=0;
				t.params=NULL;
				if(*c=='/')
				{
					NextChar();
					char* temp=c;
					ParseParams(t,start,true);
					c=temp;
					if(t.params_count>0)
						t.params=new std::string*[t.params_count];
					ParseParams(t,start,false);
				}else if(*c==',')
				{
					char buf[2550];
					sprintf(buf,"(line %i col %i) Error in command, '/' or newline or end of file expected",line,col,sizeof(token)-1);
					//printf(buf);
					throw string(buf);
				}
				tokens.Push(t);
			}
		}
	}
	~TATPTokenizer()
	{
		for(int i=0;i<tokens.GetCount();i++)
			delete[] tokens[i].params;
	}
};