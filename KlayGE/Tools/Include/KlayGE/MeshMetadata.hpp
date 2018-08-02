/**
 * @file MeshMetadata.hpp
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

#ifndef KLAYGE_TOOLS_IMAGE_CONV_TEX_METADATA_HPP
#define KLAYGE_TOOLS_IMAGE_CONV_TEX_METADATA_HPP

#pragma once

#include <KlayGE/PreDeclare.hpp>
#include <KFL/Math.hpp>

#include <string>
#include <vector>

#include <KlayGE/ToolCommon.hpp>

namespace KlayGE
{
	class KLAYGE_TOOL_API MeshMetadata
	{
	public:
		MeshMetadata();
		explicit MeshMetadata(std::string_view name);

		void Load(std::string_view name);

		void UpdateTransforms();

		bool AutoCenter() const
		{
			return auto_center_;
		}

		float3 const & Pivot() const
		{
			return pivot_;
		}
		float3 const & Translation() const
		{
			return translation_;
		}
		Quaternion const & Rotation() const
		{
			return rotation_;
		}
		float3 const & Scale() const
		{
			return scale_;
		}

		uint8_t AxisMapping(uint32_t axis) const
		{
			return axis_mapping_[axis];
		}

		uint32_t NumLods() const;
		std::string_view LodFileName(uint32_t lod) const;

		float4x4 const & Transform() const
		{
			return transform_;
		}
		float4x4 const & TransformIT() const
		{
			return transform_it_;
		}

	private:
		bool auto_center_ = false;

		float3 pivot_ = float3::Zero();
		float3 translation_ = float3::Zero();
		Quaternion rotation_ = Quaternion::Identity();
		float3 scale_ = float3(1, 1, 1);
		uint8_t axis_mapping_[3] = { 0, 1, 2 };
		std::vector<std::string> lod_file_names_;

		float4x4 transform_ = float4x4::Identity();
		float4x4 transform_it_ = float4x4::Identity();
	};
}

#endif		// KLAYGE_TOOLS_IMAGE_CONV_TEX_METADATA_HPP
