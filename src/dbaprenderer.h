/******************************************************************************
 * Copyright © 2012-2014 Institut für Nachrichtentechnik, Universität Rostock *
 * Copyright © 2006-2012 Quality & Usability Lab,                             *
 *                       Telekom Innovation Laboratories, TU Berlin           *
 *                                                                            *
 * This file is part of the SoundScape Renderer (SSR).                        *
 *                                                                            *
 * The SSR is free software:  you can redistribute it and/or modify it  under *
 * the terms of the  GNU  General  Public  License  as published by the  Free *
 * Software Foundation, either version 3 of the License,  or (at your option) *
 * any later version.                                                         *
 *                                                                            *
 * The SSR is distributed in the hope that it will be useful, but WITHOUT ANY *
 * WARRANTY;  without even the implied warranty of MERCHANTABILITY or FITNESS *
 * FOR A PARTICULAR PURPOSE.                                                  *
 * See the GNU General Public License for more details.                       *
 *                                                                            *
 * You should  have received a copy  of the GNU General Public License  along *
 * with this program.  If not, see <http://www.gnu.org/licenses/>.            *
 *                                                                            *
 * The SSR is a tool  for  real-time  spatial audio reproduction  providing a *
 * variety of rendering algorithms.                                           *
 *                                                                            *
 * http://spatialaudio.net/ssr                           ssr@spatialaudio.net *
 ******************************************************************************/

/// @file
/// Ambisonics Amplitude Panning renderer.

#ifndef SSR_DBAPRENDERER_H
#define SSR_DBAPRENDERER_H

#include "ssr_global.h"
#include "loudspeakerrenderer.h"
#include "apf/combine_channels.h"

namespace ssr
{

class DbapRenderer : public SourceToOutput<DbapRenderer, LoudspeakerRenderer>
{
  private:
    using _base = SourceToOutput<DbapRenderer, ssr::LoudspeakerRenderer>;

  public:
    static const char* name() { return "DBAP-Renderer"; }

    class Source;
    class SourceChannel;
    class Output;
    class RenderFunction;

    explicit DbapRenderer(const apf::parameter_map& params)
      : _base(params)
    {
    }

    APF_PROCESS(DbapRenderer, _base)
    {
      _process_list(_source_list);
    }

//    void load_reproduction_setup();
      
};

class DbapRenderer::Source : public _base::Source
{
  public:
    Source(const Params& p)
      : _base::Source(p, p.parent->get_output_list().size(), this)
    {}

    bool get_output_levels(sample_type* first, sample_type* last) const;
};

class DbapRenderer::SourceChannel
{
  public:
    explicit SourceChannel(const Source* s)
      : source(*s)
    {}

    const Source& source;

    using iterator = decltype(source.begin());

    iterator begin() const { return source.begin(); }
    iterator end() const { return source.end(); }

    apf::BlockParameter<sample_type> stored_weight;
};


bool
DbapRenderer::Source::get_output_levels(sample_type* first
    , sample_type* last) const
{
  assert(
      static_cast<sourcechannels_t::size_type>(std::distance(first, last))
      == this->sourcechannels.size());
  (void)last;

  for (const auto& channel: this->sourcechannels)
  {
    *first = channel.stored_weight;
    ++first;
  }

  return true;
}

class DbapRenderer::RenderFunction
{
  public:
    RenderFunction(const Output& out) : _out(out) {}

    apf::CombineChannelsResult::type select(SourceChannel& in);

    sample_type operator()(sample_type in)
    {
      return in * _weight;
    }

    sample_type operator()(sample_type in, sample_type index)
    {
      return in * _interpolator(index);
    }

  private:
    sample_type _weight;
    apf::math::linear_interpolator<sample_type> _interpolator;
    
    float square_law = 6.0f / (20.0f * log10 (2.0f));

    const Output& _out;
};

class DbapRenderer::Output : public _base::Output
{
  public:
    Output(const Params& p)
      : _base::Output(p)
      , _combiner(this->sourcechannels, this->buffer)
    {
      // TODO: add delay line (in some base class?)

      // TODO: amplitude correction for misplaced loudspeakers?
    }

    APF_PROCESS(Output, _base::Output)
    {
      _combiner.process(RenderFunction(*this));
    }

  private:
    apf::CombineChannelsInterpolation<apf::cast_proxy<SourceChannel
      , sourcechannels_t>, buffer_type> _combiner;
};

//void
//DbapRenderer::load_reproduction_setup()
//{
//  // TODO: find a way to avoid overwriting load_reproduction_setup()
//
//  _base::load_reproduction_setup();
//
//  // TODO: move some stuff to base class?
//
//  // TODO: check somehow if loudspeaker setup is reasonable?
//
//  // TODO: get loudspeaker delays from setup?
//  // delay_samples = size_t(delay * sample_rate + 0.5f)
//
//  int normal_loudspeakers = 0;
//      std::cout<<"normal_loudspeakers"<<normal_loudspeakers<<std::endl;
//  for (const auto& out: rtlist_proxy<Output>(this->get_output_list()))
//  {
//    if (out.model == Loudspeaker::subwoofer)
//    {
//      // TODO: something
//    }
//    else  // loudspeaker type == normal
//    {
//      ++normal_loudspeakers;
//      //std::cout<<"normal_loudspeakers"<<normal_loudspeakers<<std::endl;
//    }
//  }
//
//  if (normal_loudspeakers < 1)
//  {
//    throw std::logic_error("No loudspeakers found!");
//  }
//
//}


apf::CombineChannelsResult::type
DbapRenderer::RenderFunction::select(SourceChannel& in)
{
  auto weighting_factor = sample_type();
   float coeff = 0.0f;
   float safety_distance = 0.01f;

      for (auto& out : rtlist_proxy<Output>(_out.parent.get_output_list()))
      {
          // swap x and y (somehow they are mixed up between
          // coordinates of loudspeakers and sources)
          // and change sign of y (don't know why)
          Position ls_pos(out.position);
          ls_pos.y = out.position.x;
          ls_pos.x = -out.position.y;

        //auto src_pos = in.source.position;
        Position src_pos = in.source.position;
        // float source_ls_distance = (ls_pos - src_pos).length(); 
        float source_ls_distance = std::sqrt(std::pow((ls_pos.x - src_pos.x), 2.0f) + 
                                   std::pow((ls_pos.y - src_pos.y), 2.0f));

        if (source_ls_distance < safety_distance)
        {
          source_ls_distance = safety_distance;
        }
  
        float coeff_source_loudspeaker_single = (1.0f / std::pow(source_ls_distance, 
                  (2.0f * square_law)));

        coeff += coeff_source_loudspeaker_single; 
      }
      coeff = 1.0f / std::sqrt(coeff);
      
        if (_out.model == Loudspeaker::normal)
        {
          Position p(_out.position);
          p.y = _out.position.x;
          p.x = -_out.position.y;

          //float src_ls_dist = (p - in.source.position).length();
          // auto current_src_pos = in.source.position;
          Position current_src_pos = in.source.position;
          float src_ls_dist = std::sqrt(std::pow((p.x - current_src_pos.x), 2.0f) + 
                          std::pow((p.y - current_src_pos.y), 2.0f));

          if (src_ls_dist < safety_distance)
          {
            src_ls_dist = safety_distance;
          }

          weighting_factor = coeff / (std::pow(src_ls_dist, square_law));
        }
        else if (_out.model == Loudspeaker::subwoofer)
        {
          // TODO: subwoofer gets weighting factor 1.0?
          weighting_factor = 1;
        }
        else
        {
          // TODO
        }

  // Apply source volume, mute, ...
  weighting_factor *= in.source.weighting_factor;
//std::cout<<"weighting_factor"<<weighting_factor<<std::endl;

  in.stored_weight = weighting_factor;

  auto old_weight = in.stored_weight.old();

  using namespace apf::CombineChannelsResult;

  if (old_weight == 0 && weighting_factor == 0)
  {
    return nothing;
  }
  else if (old_weight == weighting_factor)
  {
    _weight = weighting_factor;
    return constant;
  }
  else
  {
    _interpolator.set(old_weight, weighting_factor, _out.parent.block_size());
    return change;
  }
}

}  // namespace ssr

#endif

// Settings for Vim (http://www.vim.org/), please do not remove:
// vim:softtabstop=2:shiftwidth=2:expandtab:textwidth=80:cindent
