#include "bitChar.h"

bitChar::bitChar()
{
	shift_count = 0;
	c = (unsigned char*)calloc(1, sizeof(char));
}

void bitChar::setBITS(std::string _X)
{
	BITS = _X;
}

//Returns number of bits inserted
int bitChar::insertBits(std::ofstream& outf)
{
	int total = 0;

	while(BITS.length())
	{
		if(BITS[0] == '1')
			*c |= 1;
		*c <<= 1;
		++shift_count;
		++total;
		BITS.erase(0, 1);

		if(shift_count == 7)
		{
			writeBits(outf);
			shift_count = 0;
			free(c);
			c = (unsigned char*)calloc(1, sizeof(char));
		}
	}

	//Account for any trailing bits and push them over
	if(shift_count > 0)
	{
		*c <<= (8 - shift_count);
		writeBits(outf);
		free(c);
		c = (unsigned char*)calloc(1, sizeof(char));
	}

	return total;
}

//Outputs a char in binary format
std::string bitChar::getBits(unsigned char _X)
{
	std::stringstream _itoa;

	int _size = sizeof(unsigned char) * 8;

	for(unsigned _s = 0; _s < _size - 1; ++_s)
	{
		_itoa << ((_X >> (_size - 1 - _s)) & 1);
	}

	return _itoa.str();
}

void bitChar::writeBits(std::ofstream& outf)
{
	outf << *c;
}

bitChar::~bitChar()
{
	if(c)
		free(c);
}
