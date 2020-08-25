#pragma once
#include <Corrade/PluginManager/Manager.h>
#include <Corrade/Utility/FormatStl.h>
#include <Corrade/Utility/Resource.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/Complex.h>
#include <Magnum/Math/Matrix3.h>
#include <Magnum/Shaders/DistanceFieldVector.h>
#include <Magnum/Text/AbstractFont.h>
#include <Magnum/Text/DistanceFieldGlyphCache.h>
#include <Magnum/Text/Renderer.h>
#include <string>

namespace DosAutonomy
{

class panelFontRender
{
public:
	panelFontRender(int viewWidth, int viewHeight)
		: m_cache{ Magnum::Vector2i{2048}, Magnum::Vector2i{512}, 22 }
	{
		/* Load a TrueTypeFont plugin and open the font */
		Magnum::Utility::Resource rs("fonts");
		m_font = m_manager.loadAndInstantiate("TrueTypeFont");
		if (!m_font || !m_font->openData(rs.getRaw("SourceSansPro-Regular.ttf"), 180.0f))
		{
			Magnum::Fatal{} << "Cannot open font file";
		}

		/* Glyphs we need to render everything */
		m_font->fillGlyphCache(m_cache,
			"abcdefghijklmnopqrstuvwxyz"
			"ABCDEFGHIJKLMNOPQRSTUVWXYZ"
			"0123456789:-+,.! ");

		// Dynamic text for printing on screen
		m_dynamicText.reset(new Magnum::Text::Renderer2D(*m_font, m_cache, 32.0f, Magnum::Text::Alignment::TopRight));
		m_dynamicText->reserve(40, Magnum::GL::BufferUsage::DynamicDraw, Magnum::GL::BufferUsage::StaticDraw);
		m_textXform =
			Magnum::Matrix3::projection(Magnum::Vector2{ float(viewWidth), float(viewHeight) }) *
			Magnum::Matrix3::translation(Magnum::Vector2{ float(viewWidth), float(viewHeight) }*0.5f);
	}

	void draw()
	{
		updateText();
		m_shader.bindVectorTexture(m_cache.texture());

		m_shader
			.setTransformationProjectionMatrix(m_textXform)
			.setColor(Magnum::Color4(1.0f, 1.0f, 1.0f, 1.0f))
			.setOutlineRange(0.5f, 1.0f)
			.setSmoothness(0.075f);
		m_dynamicText->mesh().draw(m_shader);
	}

	void updateText()
	{
		m_dynamicText->render(m_textStr);
	}


public:
	Magnum::PluginManager::Manager<Magnum::Text::AbstractFont> m_manager;
	Magnum::Containers::Pointer<Magnum::Text::AbstractFont> m_font;

	Magnum::Text::DistanceFieldGlyphCache m_cache;
	Magnum::GL::Mesh _rotatingText{ Magnum::NoCreate };
	Magnum::GL::Buffer m_vertices, m_indices;
	Magnum::Containers::Pointer<Magnum::Text::Renderer2D> m_dynamicText;
	Magnum::Shaders::DistanceFieldVector2D m_shader;
	Magnum::Matrix3 m_textXform;
	std::string m_textStr = "Hello World";
};
 
} // end of namespace DosAutonomy