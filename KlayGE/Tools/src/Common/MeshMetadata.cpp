/**
 * @file MeshMetadata.cpp
 * @author Minmin Gong
 *
 * @section DESCRIPTION
 *
 * This source file is part of KlayGE
 * For the latest info, see http://www.klayge.org
 *
 * @section LICENSE
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * You may alternatively use this source under the terms of
 * the KlayGE Proprietary License (KPL). You can obtained such a license
 * from http://www.klayge.org/licensing/.
 */

#include <KlayGE/KlayGE.hpp>
#include <KFL/ErrorHandling.hpp>
#include <KFL/Hash.hpp>
#include <KlayGE/ResLoader.hpp>

#include <algorithm>
#include <iterator>

#include <rapidjson/document.h>

#include <KlayGE/MeshMetadata.hpp>

namespace
{
	float GetFloat(rapidjson::Value const & value)
	{
		if (value.IsFloat())
		{
			return value.GetFloat();
		}
		else if (value.IsDouble())
		{
			return static_cast<float>(value.GetDouble());
		}
		else if (value.IsInt())
		{
			return static_cast<float>(value.GetInt());
		}
		else if (value.IsUint())
		{
			return static_cast<float>(value.GetUint());
		}
		else if (value.IsInt64())
		{
			return static_cast<float>(value.GetInt64());
		}
		else if (value.IsUint64())
		{
			return static_cast<float>(value.GetUint64());
		}
		else
		{
			KFL_UNREACHABLE("Invalid value type.");
		}
	}

	int GetInt(rapidjson::Value const & value)
	{
		if (value.IsInt())
		{
			return value.GetInt();
		}
		else if (value.IsUint())
		{
			return static_cast<int>(value.GetUint());
		}
		else if (value.IsInt64())
		{
			return static_cast<int>(value.GetInt64());
		}
		else if (value.IsUint64())
		{
			return static_cast<int>(value.GetUint64());
		}
		else
		{
			KFL_UNREACHABLE("Invalid value type.");
		}
	}
}

namespace KlayGE
{
	MeshMetadata::MeshMetadata()
	{
	}

	MeshMetadata::MeshMetadata(std::string_view name)
	{
		this->Load(name);
	}

	void MeshMetadata::Load(std::string_view name)
	{
		MeshMetadata new_metadata;

		ResIdentifierPtr metadata_file = ResLoader::Instance().Open(name);
		if (metadata_file)
		{
			std::string metadata;
			std::copy(std::istream_iterator<char>(metadata_file->input_stream()), std::istream_iterator<char>(),
				std::back_inserter<std::string>(metadata));

			rapidjson::Document document;
			document.Parse(metadata.data());
			BOOST_ASSERT(!document.HasParseError());

			uint32_t const version = document["version"].GetUint();
			Verify(version == 1);

			if (document.HasMember("auto_center"))
			{
				auto const & auto_center_val = document["auto_center"];
				BOOST_ASSERT(auto_center_val.IsBool());
				new_metadata.auto_center_ = auto_center_val.GetBool();
			}

			if (document.HasMember("pivot"))
			{
				auto const & pivot_val = document["pivot"];
				BOOST_ASSERT(pivot_val.IsArray());
				BOOST_ASSERT(pivot_val.Size() >= new_metadata.pivot_.size());
				uint32_t index = 0;
				for (auto iter = pivot_val.Begin(); (iter != pivot_val.End()) && (index < new_metadata.pivot_.size()); ++ iter, ++ index)
				{
					BOOST_ASSERT(iter->IsNumber());
					new_metadata.pivot_[index] = GetFloat(*iter);
				}
			}

			if (document.HasMember("translation"))
			{
				auto const & translation_val = document["translation"];
				BOOST_ASSERT(translation_val.IsArray());
				BOOST_ASSERT(translation_val.Size() >= new_metadata.translation_.size());
				uint32_t index = 0;
				for (auto iter = translation_val.Begin(); (iter != translation_val.End()) && (index < new_metadata.translation_.size());
					++ iter, ++ index)
				{
					BOOST_ASSERT(iter->IsNumber());
					new_metadata.translation_[index] = GetFloat(*iter);
				}
			}

			if (document.HasMember("rotation"))
			{
				auto const & rotation_val = document["rotation"];
				BOOST_ASSERT(rotation_val.IsArray());
				BOOST_ASSERT(rotation_val.Size() >= new_metadata.rotation_.size());
				uint32_t index = 0;
				for (auto iter = rotation_val.Begin(); (iter != rotation_val.End()) && (index < new_metadata.rotation_.size());
					++ iter, ++ index)
				{
					BOOST_ASSERT(iter->IsNumber());
					new_metadata.rotation_[index] = GetFloat(*iter);
				}
			}

			if (document.HasMember("scale"))
			{
				auto const & scale_val = document["scale"];
				BOOST_ASSERT(scale_val.IsArray());
				BOOST_ASSERT(scale_val.Size() >= new_metadata.scale_.size());
				uint32_t index = 0;
				for (auto iter = scale_val.Begin(); (iter != scale_val.End()) && (index < new_metadata.scale_.size()); ++ iter, ++ index)
				{
					BOOST_ASSERT(iter->IsNumber());
					new_metadata.scale_[index] = GetFloat(*iter);
				}
			}

			if (document.HasMember("axis_mapping"))
			{
				auto const & axis_mapping_val = document["axis_mapping"];
				BOOST_ASSERT(axis_mapping_val.IsArray());
				BOOST_ASSERT(axis_mapping_val.Size() == 3);
				uint32_t index = 0;
				for (auto iter = axis_mapping_val.Begin(); (iter != axis_mapping_val.End()) && (index < 3); ++ iter, ++ index)
				{
					BOOST_ASSERT(iter->IsInt() || iter->IsUint() || iter->IsInt64() || iter->IsUint64());
					new_metadata.axis_mapping_[index] = static_cast<uint8_t>(GetInt(*iter));
				}
			}

			if (document.HasMember("lod"))
			{
				auto const & lod_val = document["lod"];
				BOOST_ASSERT(lod_val.IsArray());
				new_metadata.lod_file_names_.resize(lod_val.Size());
				uint32_t index = 0;
				for (auto iter = lod_val.Begin(); iter != lod_val.End(); ++ iter)
				{
					BOOST_ASSERT(iter->IsString());
					new_metadata.lod_file_names_[index] = iter->GetString();
				}
			}
		}

		*this = std::move(new_metadata);
	}
	
	void MeshMetadata::UpdateTransforms()
	{
		transform_ = MathLib::transformation(&pivot_, static_cast<Quaternion*>(nullptr), &scale_,
			&pivot_, &rotation_, &translation_);

		const float4 axis[3] = { transform_.Col(0), transform_.Col(1), transform_.Col(2) };
		transform_.Col(0, axis[axis_mapping_[0]]);
		transform_.Col(1, axis[axis_mapping_[1]]);
		transform_.Col(2, axis[axis_mapping_[2]]);

		transform_it_ = MathLib::transpose(MathLib::inverse(transform_));
	}

	uint32_t MeshMetadata::NumLods() const
	{
		return lod_file_names_.empty() ? 1 : static_cast<uint32_t>(lod_file_names_.size());
	}

	std::string_view MeshMetadata::LodFileName(uint32_t lod) const
	{
		return lod_file_names_[lod];
	}
}
