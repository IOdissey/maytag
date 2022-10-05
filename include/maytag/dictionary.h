#pragma once

#include <cstdint>
#include <limits>

#include "tag_family.h"
#include "tag.h"


namespace maytag::_
{
	class Dictionary
	{
	private:
		struct quick_decode_t
		{
			uint64_t code = std::numeric_limits<uint64_t>::max(); // The queried code.
			uint16_t id;     // The tag ID (a small integer).
			uint8_t hamming; // How many errors corrected?
		};

		const uint32_t _nbits;
		const uint32_t _ncodes;
		const uint64_t* const _codes;
		const uint64_t _mask; 
		quick_decode_t* _dictionary = nullptr;
		uint32_t _size = 0;
		uint8_t _max_hamming = 255;
		uint32_t _max_search = 0;

		inline uint32_t _hash(uint64_t code) const
		{
			return (code * 131) % _size;
		}

		void _add(uint64_t code, uint16_t id, uint8_t hamming)
		{
			uint32_t n = 1;
			uint32_t hash = _hash(code);
			while (_dictionary[hash].code != std::numeric_limits<uint64_t>::max())
			{
				hash = (hash + 1) % _size;
				++n;
			}
			if (n > _max_search)
				_max_search = n;
			_dictionary[hash].code = code;
			_dictionary[hash].id = id;
			_dictionary[hash].hamming = hamming;
		}

		// Assuming we are drawing the image one quadrant at a time, what would the rotated image look like?
		// Special care is taken to handle the case where there is a middle pixel of the image.
		uint64_t _rotate90(uint64_t w) const
		{
			if (_nbits % 4 == 1)
			{
				const uint32_t p = (_nbits - 1) >> 2;
				w = ((w >> 1) << (p + 1)) | ((w >> (3 * p + 1)) << 1) | (w & 1);
			}
			else
			{
				const uint32_t p = _nbits >> 2;
				w = (w << p) | (w >> (3 * p));
			}
			return w & _mask;
		}

		//
		bool _create(uint8_t hamming, double size_scale)
		{
			if (hamming > 3)
				hamming = 3;
			if (hamming <= _max_hamming && _max_hamming != 255)
				return false;
			_max_hamming = hamming;
			uint32_t capacity = _ncodes;
			if (_max_hamming >= 1)
				capacity += _ncodes * _nbits;
			if (_max_hamming >= 2)
				capacity += _ncodes * _nbits * (_nbits - 1) / 2;
			if (_max_hamming >= 3)
				capacity += _ncodes * _nbits * (_nbits - 1) * (_nbits - 2) / 6;
			_size = static_cast<uint32_t>(capacity * size_scale);
			if (_size < capacity)
				_size = capacity;
			if (_dictionary)
				delete[] _dictionary;
			_dictionary = new quick_decode_t[_size];
			_max_search = 0;
			//
			const uint64_t one = 1;
			for (uint32_t i = 0; i < _ncodes; ++i)
			{
				const uint64_t code = _codes[i];
				_add(code, i, 0);
			}
			if (_max_hamming >= 1)
			{
				for (uint32_t i = 0; i < _ncodes; ++i)
				{
					const uint64_t code = _codes[i];
					for (uint32_t j = 0; j < _nbits; ++j)
						_add(code ^ (one << j), i, 1);
				}
			}
			if (_max_hamming >= 2)
			{
				for (uint32_t i = 0; i < _ncodes; ++i)
				{
					const uint64_t code = _codes[i];
					for (uint32_t j = 0; j < _nbits; ++j)
					{
						for (uint32_t k = 0; k < j; ++k)
							_add(code ^ (one << j) ^ (one << k), i, 2);
					}
				}
			}
			if (_max_hamming >= 3)
			{
				for (uint32_t i = 0; i < _ncodes; ++i)
				{
					const uint64_t code = _codes[i];
					for (uint32_t j = 0; j < _nbits; ++j)
					{
						for (uint32_t k = 0; k < j; ++k)
						{
							for (uint32_t m = 0; m < k; ++m)
								_add(code ^ (one << j) ^ (one << k) ^ (one << m), i, 3);
						}
					}
				}
			}
			return true;
		}

		void _print_stat(const std::string& name, const std::string& text, double size_scale) const
		{
			std::cout << "MayTag dictionary " << text << "\n"
				<< "\tname: " << name << "\n"
				<< "\tncodes: " << _ncodes << "\n"
				<< "\thamming: " << static_cast<int>(_max_hamming) << "\n"
				<< "\tmax_search: " << _max_search << "\n"
				<< "\tsize_scale: " << size_scale << "\n"
				<< "\tsize: " << _size * sizeof(quick_decode_t) << " B" << std::endl;
		}

	public:
		Dictionary(const tag_family_t& family, double size_scale = 3.0, bool stat = false):
			_nbits(family.nbits),
			_ncodes(family.ncodes),
			_codes(family.codes),
			_mask(((uint64_t)1 << family.nbits) - 1)
		{
			if (_create(family.hamming, size_scale) && stat)
				_print_stat(family.name, "created", size_scale);
		}

		~Dictionary()
		{
			delete[] _dictionary;
		}

		void update_hamming(const tag_family_t& family, double size_scale = 3.0, bool stat = false)
		{
			if (_create(family.hamming, size_scale) && stat)
				_print_stat(family.name, "updated", size_scale);
		}

		bool decode(uint64_t code, uint16_t& id, uint8_t& hamming, uint8_t& rot) const
		{
			for (rot = 0; rot < 4; ++rot)
			{
				if (rot > 0)
					code = _rotate90(code);
				uint32_t hash = _hash(code);
				for (uint32_t i = 0; i < _max_search; ++i)
				{
					const uint64_t dcode = _dictionary[hash].code;
					if (dcode == std::numeric_limits<uint64_t>::max())
						break;
					else if (dcode == code)
					{
						id = _dictionary[hash].id;
						hamming = _dictionary[hash].hamming;
						return true;
					}
					hash = (hash + 1) % _size;
				}
			}
			return false;
		}
	};
}