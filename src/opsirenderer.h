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
/// Optimised Phantom Source Imaging renderer

#ifndef SSR_OPSIRENDERER_H
#define SSR_OPSIRENDERER_H

#include "loudspeakerrenderer.h"
#include "ssr_global.h"

#include "apf/convolver.h"  // for apf::conv::...
#include "apf/blockdelayline.h"  // for NonCausalBlockDelayLine
#include "apf/sndfiletools.h"  // for apf::load_sndfile
#include "apf/combine_channels.h"  // for apf::raised_cosine_fade, ...
#include "apf/biquad.h" // for apf::BiQuad and apf::SosCoefficients

// TODO: make more flexible option:
#define WEIGHTING_OLD
//#define WEIGHTING_DELFT

namespace ssr
{


class OpsiRenderer : public SourceToOutput<OpsiRenderer, LoudspeakerRenderer>
{
  private:
    using _base = SourceToOutput<OpsiRenderer, ssr::LoudspeakerRenderer>;

  public:
    static const char* name() { return "OPSI-Renderer"; }

    class Input;
    class Source;
    class SourceChannel;
    class Output;
    class RenderFunction;

    OpsiRenderer(const apf::parameter_map& params)
      : _base(params)
      , _fade(this->block_size())
      , _max_delay(this->params.get("delayline_size", 0))
      , _initial_delay(this->params.get("initial_delay", 0))
      , _max_angle(params.get("vbap_max_angle", apf::math::deg2rad(180.0)))
      , incidence_angle(0.0f)
    {
      // TODO: compute "ideal" initial delay?
      // TODO: check if given initial delay is sufficient?

      // TODO: make option --prefilter=none?

      // TODO: get pre-filter from reproduction setup!
      // TODO: allow alternative files for different sample rates

      SndfileHandle prefilter;
      try
      {
        prefilter = apf::load_sndfile(
            this->params.get("prefilter_file", ""), this->sample_rate(), 1);
      }
      catch (const std::logic_error& e)
      {
        throw std::logic_error(
            "Error loading OPSI pre-equalization filter file: "
            + std::string(e.what()));
      }

      size_t size = prefilter.frames();

      auto ir = apf::fixed_vector<sample_type>(size);

      size = prefilter.readf(ir.data(), size);

      // TODO: warning if size changed?
      // TODO: warning if size == 0?

      _pre_filter.reset(new apf::conv::Filter(this->block_size()
            , ir.begin(), ir.end()));
    }

    APF_PROCESS(OpsiRenderer, _base)
    {
      this->_process_list(_source_list);
    }

    void load_reproduction_setup();


  private:
    apf::raised_cosine_fade<sample_type> _fade;
    std::unique_ptr<apf::conv::Filter> _pre_filter;

    size_t _max_delay, _initial_delay;

    struct LoudspeakerEntry
    {
      // Note: This is non-explicit to allow comparison with angle
      LoudspeakerEntry(float angle_, bool valid_section_ = false
          , const Output* output_ = nullptr)
        : angle(angle_)
        , valid_section(valid_section_)
        , ls_ptr(output_)
      {}

      // Comparison operator is needed for std::sort() and std::upper_bound()
      friend
      bool operator<(const LoudspeakerEntry& lhs, const LoudspeakerEntry& rhs)
      {
        return lhs.angle < rhs.angle;
      }

      float angle;  // radians
      bool valid_section;  // Is the angle to the next loudspeaker < _max_angle?
      const Output* ls_ptr;
    };

    std::vector<LoudspeakerEntry> _sorted_loudspeakers;
    apf::BlockParameter<Position> _reference_offset_position;
    Position _absolute_reference_offset_position;
    float _max_angle;
    void _update_angles();
    void _sort_loudspeakers();
    void _update_valid_sections();

    float incidence_angle;

};

class OpsiRenderer::Input : public _base::Input
{
  public:
    friend class Source;  // give access to _delayline

    Input(const Params& p)
      : _base::Input(p)
      // TODO: check if _pre_filter != 0!
      , _convolver(*this->parent._pre_filter)
      , _delayline(this->parent.block_size(), this->parent._max_delay
          , this->parent._initial_delay)
    {}

    APF_PROCESS(Input, _base::Input)
    {
      _convolver.add_block(this->buffer.begin());
      _delayline.write_block(_convolver.convolve());
    }

  private:
    apf::conv::StaticConvolver _convolver;
    apf::NonCausalBlockDelayLine<sample_type> _delayline;
};

class OpsiRenderer::SourceChannel : public apf::has_begin_and_end<
                          apf::NonCausalBlockDelayLine<sample_type>::circulator>
{
  public:
    SourceChannel(const Source& s)
      : crossfade_mode(0)
      , weighting_factor(0.0f)
      , delay(0)
      , source(s)
    {}

    void update();

    int crossfade_mode;
    apf::BlockParameter<sample_type> weighting_factor;
    apf::BlockParameter<int> delay;

    const Source& source;

    // TODO: avoid making those public:
    using apf::has_begin_and_end<apf::NonCausalBlockDelayLine<sample_type>
      ::circulator>::_begin;
    using apf::has_begin_and_end<apf::NonCausalBlockDelayLine<sample_type>
      ::circulator>::_end;
};

class OpsiRenderer::RenderFunction
{
  public:
    RenderFunction(const Output& out, float source_angle, 
    const LoudspeakerEntry& first, 
    const LoudspeakerEntry& second) 
    : _in(0)
    , _out(out) 
    , _source_angle(source_angle)
    , _first(first)
    , _second(second)
    , a0(1.0)
    , a1(-1.6828206600315097)
    , a2(0.7535314157309236)
    , highpass_b0(0.8590880189406084)
    , highpass_b1(-1.7181760378812168)
    , highpass_b2(0.8590880189406084)
    , lowpass_b0(0.017677688924853493)
    , lowpass_b1(0.035355377849706986)
    , lowpass_b2(0.017677688924853493)
    , filter_typ(1)
    {}

    apf::CombineChannelsResult::type select(SourceChannel& in);

    apf::BiQuad<double> tiefpass = apf::BiQuad<double>();
    apf::BiQuad<double> hochpass = apf::BiQuad<double>();


    sample_type operator()(sample_type in)
    {   
      in_clean = in; // for highpass filtered part (panning)

        // using namespace apf::math;
        //static int funktionsaufruf_biquad = 0;
             
             sample_type in_pan = 0;

                    // calculate coefficients
                    biquad_filtering(); 
                        
                    // First version of BiquadFilter using apf::BiQuad
//                  auto c_l = apf::SosCoefficients<double>(lowpass_b0, 
//                            lowpass_b1, lowpass_b2, a1, a2);
//                  tiefpass = c_l;
//                  in = tiefpass(in) * _new_factor;

                    // Second version of BiquadFilter
                    out_current = lowpass_b0 * in 
                        + lowpass_b1 * in_old_1 
                        + lowpass_b2 * in_old_2
                        - a1 * out_old_1 
                        - a2 * out_old_2;
                   
                    out_old_2 = out_old_1;
                    out_old_1 = out_current;
                    in_old_2 = in_old_1;
                    in_old_1 = in;

                    in = out_current * _new_factor;
                    // in = in_clean * _new_factor; // apply  no filter

         if (_first.valid_section) // Additional Amplitude Panning
         {
         //filter_typ = 0;
         //std::cout<<"1"<<std::endl;
              using namespace apf::math;

              float phi_0 = wrap_two_pi(_second.angle - _first.angle) / 2.0f;

              float phi = wrap_two_pi(_source_angle - _first.angle - phi_0);

              float num1 = std::cos(phi) * std::sin(phi_0);
              float num2 = std::sin(phi) * std::cos(phi_0);
              float den  = 2 * std::cos(phi_0) * std::sin(phi_0);

                  auto c_h = apf::SosCoefficients<double>(highpass_b0, 
                            highpass_b1, highpass_b2, a1, a2);
                        hochpass = c_h;
                
                // TODO: Get loop working
                // How to use get_output_list() here? 
//              for (const auto& out: rtlist_proxy<Output>(parent.get_output_list()))
//              {
                // TODO: handle subwoofers!

//                if (_first.ls_ptr == &out) //_out ?
//                {
//                  float first_weight = (num1 - num2) / den;
//                  // in_pan = in_clean * first_weight;     // testing: without highpass
//                  in_pan = hochpass(in_clean) * first_weight;
                  //return (in * _new_factor * first_weight);
                  //return (in * _new_factor); // TEST
//                }
//                else if (_second.ls_ptr == &out) //_out ?
//                {
//                  float second_weight = (num1 + num2) / den;
//                  // in_pan = in_clean * second_weight;     // testing: without highpass
//                  in_pan = hochpass(in_clean) * second_weight;
                  //return (in * _new_factor * second_weight);
                  //return (in * _new_factor); // TEST
//                }
//              }
         }
         //else
         //{
          // Do nothing
         //}
//         in = in * _new_factor;
    return in + in_pan;
    }
    

    sample_type operator()(sample_type in, apf::fade_out_tag)
    {
                        biquad_filtering();

                        auto c_l = apf::SosCoefficients<double>(lowpass_b0, 
                            lowpass_b1, lowpass_b2, a1, a2);
                        tiefpass = c_l;

                        in = tiefpass(in);

     // std::cout<<"op2"<<std::endl; // Console
      return in * _old_factor;
    }

    void update()
    {
      assert(_in);
      _in->update();
    }

    void biquad_filtering();

  private:
    sample_type _old_factor, _new_factor;

    SourceChannel* _in;
    const Output& _out;

    float _source_angle;
    const LoudspeakerEntry& _first;
    const LoudspeakerEntry& _second;

  // General coefficients denominator
   double a0;
   double a1;
   double a2;
  
  // Coefficients Highpass
   double highpass_b0;
   double highpass_b1;
   double highpass_b2;

  // Coefficients Lowpass
   double lowpass_b0;
   double lowpass_b1;
   double lowpass_b2;

   int filter_typ;
  // int funktionsaufruf_biquad;

   sample_type in_old_2 = in_old_1;
   sample_type in_old_1 = in_clean;
   sample_type out_old_2 = out_old_1;
   sample_type out_old_1 = out_current;
   sample_type out_current = 0;
   sample_type in_clean;
   // TODO: find another way to define variables 
   // to avoid crackling sound.
   // Supposed problem: variables get initialised 
   // by calling _combiner.process(RenderFunction(etc.));
   // which is done for each buffer (?) of 1024 samples.
   // Therefore the first two/three samples do not match (coefficients = 0)
   // and there is an interruption of the waveform.
   // Solution which did not work out: define global variables
   // and access them either by parent-pointer or using of namespace

};

class OpsiRenderer::Output : public _base::Output
{
  public:
    friend class Source;  // to be able to see _sourcechannels

    Output(const Params& p)
      : _base::Output(p)
      , _combiner(this->sourcechannels, this->buffer, this->parent._fade)
    {}

    APF_PROCESS(Output, _base::Output)
    {
     // this->parent._update_valid_sections();

      auto l_begin = this->parent._sorted_loudspeakers.begin();
      auto l_end = this->parent._sorted_loudspeakers.end();

      float source_angle = this->parent.incidence_angle;


      auto second = apf::make_circular_iterator(l_begin, l_end
          , std::upper_bound(l_begin, l_end, source_angle));

      auto first = second;

      --first;
      
      // for panning LoudspeakerEntry* are handed over
      _combiner.process(RenderFunction(*this, source_angle, *first, *second));
    }

  private:
    apf::CombineChannelsCrossfade<apf::cast_proxy<SourceChannel
      , sourcechannels_t>, buffer_type
      , apf::raised_cosine_fade<sample_type>> _combiner;
};

class OpsiRenderer::Source : public _base::Source
{
  private:
    void _process();

  public:
    Source(const Params& p)
      : _base::Source(p, p.parent->get_output_list().size(), *this)
      , delayline(p.input->_delayline)
    {}

    APF_PROCESS(Source, _base::Source)
    {

      this->parent.incidence_angle = apf::math::wrap_two_pi(apf::math::deg2rad(
            ((this->position
              - this->parent._absolute_reference_offset_position).orientation()
             - this->parent.state.reference_orientation).azimuth));

      _process();
    }

    bool get_output_levels(sample_type* first, sample_type* last) const
    {
      assert(size_t(std::distance(first, last)) == this->sourcechannels.size());

      auto channel = this->sourcechannels.begin();

      for ( ; first != last; ++first)
      {
        *first = channel->weighting_factor;
        ++channel;
      }
      return true;
    }

    const apf::NonCausalBlockDelayLine<sample_type>& delayline;

    //private:
    bool _focused;
};


void
OpsiRenderer::_update_angles()
{
  for (auto& ls: _sorted_loudspeakers)
  {
    // NOTE: reference_offset_orientation doesn't affect rendering

    ls.angle = apf::math::wrap_two_pi(apf::math::deg2rad((ls.ls_ptr->position
        - _reference_offset_position).orientation().azimuth));
  }
}

void
OpsiRenderer::_sort_loudspeakers()
{
  std::sort(_sorted_loudspeakers.begin(), _sorted_loudspeakers.end());
}

void
OpsiRenderer::_update_valid_sections()
{
  //int i = 0;
  for (auto ls = _sorted_loudspeakers.begin()
      ; ls != _sorted_loudspeakers.end()
      ; ++ls)
  {
    auto next_ls = ls;
    if (++next_ls == _sorted_loudspeakers.end())
    {
      // If there is only one loudspeaker, it will always be invalid.
      next_ls = _sorted_loudspeakers.begin();
      ls->valid_section = (ls->angle + _max_angle) >= (next_ls->angle
          + apf::math::deg2rad(360.0));
    }
    else
    {
      ls->valid_section = (ls->angle + _max_angle) >= next_ls->angle;
    }
    //std::cout<<"i: "<<i<<" "<<ls[i].valid_section<<std::endl; // Console
    //i++;
  }
}

void
OpsiRenderer::load_reproduction_setup()
{
  // TODO: find a way to avoid overwriting load_reproduction_setup()

  _base::load_reproduction_setup();

  // TODO: check somehow if loudspeaker setup is reasonable?

  // TODO: get loudspeaker delays from setup?
  // delay_samples = size_t(delay * sample_rate + 0.5f)

  for (const auto& out: rtlist_proxy<Output>(this->get_output_list()))
  {
    if (out.model == Loudspeaker::subwoofer)
    {
      // TODO: put subwoofers in separate list? (for get_output_levels())
    }
    else  // loudspeaker type == normal
    {
      _sorted_loudspeakers.emplace_back(0, false, &out);
    }
  }

  if (_sorted_loudspeakers.size() < 1)
  {
    throw std::logic_error("No loudspeakers found!");
  }

  _update_angles();
  _sort_loudspeakers();
  _update_valid_sections();

        _absolute_reference_offset_position
        = Position(_reference_offset_position).rotate(
            this->state.reference_orientation)
        + this->state.reference_position;
}

void
OpsiRenderer::RenderFunction::biquad_filtering()
{
    float f0 = 0.0f;
//  float ls_spacing = OpsiRenderer::RenderFunction::calculate_loudspeaker_spacing();

//    int i = 0;
//    float loudspeaker_spacing = 0.0f;
//    for (auto& out : rtlist_proxy<Output>(_out.parent.get_output_list()))
//      {
//        Position first_pos(out.position);
//
//        if (i == 0)
//        {
//          first_pos.y = out.position.x;
//          first_pos.x = out.position.y;
//        }
//
//        Position second_pos(out.position);
//        second_pos.y = out.position.x;
//        second_pos.x = out.position.y;
//        i++;
//
//        if (first_pos != second_pos)
//        {
//        loudspeaker_spacing = (second_pos - first_pos).length();
//        std::cout<<"ls_spac: "<<loudspeaker_spacing<<std::endl;
//        break;
//        }
//      }

//  f0 = ssr::c / (2.0f * loudspeaker_spacing);

 // auto sample_rate = apf::pointer_policy::sample_rate();
 // auto sample_rate = AudioPlayer::Soundfile::_get_jack_sample_rate();

// For my purposes cut-off frequency needs to be defined manually. 
// normally it is calculated using loudspeaker spacing (algorithm above)
 f0 = 2000.0f;

 // TODO get sample_rate of jack
 float sample_rate = 44100.0f;

 // *** Berechnung nach Audio-EQ-Cookbook *** //
 // float omega0 = 2.0f * M_PI * f0/sample_rate; 

  // Weitere Zwischenvariablen
 // float q = 1.0;
 // float alpha = std::sin(omega0)/(2.0*q);

  // Allgemeine Koeffizienten Nenner
  //  a0 = 1.0 + alpha;
  //  a1 = (-2.0*cos(omega0))/a0;
  //  a2 = (1.0 - alpha)/a0;

  // Koeffizienten Hochpass berechnen
  //  highpass_b0 = ((1.0 + cos(omega0))/2.0)/a0;
  //  highpass_b1 = (-(1.0 + cos(omega0)))/a0;
  //  highpass_b2 = ((1.0 + cos(omega0))/2.0)/a0;

  // Koeffizienten Tiefpass berechnen
  //  lowpass_b0 = ((1.0 - cos(omega0))/2.0)/a0;
  //  lowpass_b1 = (1.0 - cos(omega0))/a0;
  //  lowpass_b2 = ((1.0 - cos(omega0))/2.0)/a0;
  //  a0 /= a0;

    // *** Berechnung nach Zoelzer *** //
    double k = M_PI * f0/sample_rate;
    lowpass_b0 = std::pow(k, 2) / ( 1.0 + std::sqrt(2) * k + std::pow(k, 2));
    lowpass_b1 = 2.0 * lowpass_b0;
    lowpass_b2 = lowpass_b0;

    highpass_b0 = 1.0 / ( 1.0 + std::sqrt(2) * k + std::pow(k, 2));
    highpass_b1 = -2.0 * highpass_b0;
    highpass_b2 = highpass_b0;

    a0 = 1.0;
    a1 = (2.0 * (std::pow(k, 2) - 1.0)) / ( 1.0 + std::sqrt(2) * k + std::pow(k, 2));
    a2 = (1.0 - std::sqrt(2) * k + std::pow(k, 2)) / ( 1.0 + std::sqrt(2) * k + std::pow(k, 2));

// std::cout<<"a0: "<<a0<<std::endl;
// std::cout<<"a1: "<<a1<<std::endl;
// std::cout<<"a2: "<<a2<<std::endl;
// std::cout<<"HP b1: "<<highpass_b0<<std::endl;
// std::cout<<"HP b2: "<<highpass_b1<<std::endl;
// std::cout<<"HP b3: "<<highpass_b2<<std::endl;
// std::cout<<"TP b1: "<<lowpass_b0<<std::endl;
// std::cout<<"TP b2: "<<lowpass_b1<<std::endl;
// std::cout<<"TP b3: "<<lowpass_b2<<std::endl;
}


void OpsiRenderer::Source::_process()
{
  if (this->model == ::Source::plane)
  {
    // do nothing, focused-ness is irrelevant for plane waves
    _focused = false;
  }
  else
  {
    _focused = true;
    for (const auto& out: rtlist_proxy<Output>(_input.parent.get_output_list()))
    {
      // subwoofers have to be ignored!
      if (out.model == Loudspeaker::subwoofer) continue;

      // TODO: calculate with inner product

      // angle (modulo) between the line connecting source<->loudspeaker
      // and the loudspeaker orientation

      // TODO: avoid getting reference 2 times (see select())
      auto ls = DirectionalPoint(out);
      auto ref = DirectionalPoint(out.parent.state.reference_position
          , out.parent.state.reference_orientation);
      ls.transform(ref);

      auto a = apf::math::wrap(angle(ls.position - this->position
            , ls.orientation), 2 * apf::math::pi<sample_type>());

      auto halfpi = apf::math::pi<sample_type>()/2;

      if (a < halfpi || a > 3 * halfpi)
      {
        // if at least one loudspeaker "turns its back" to the source, the
        // source is considered non-focused
        _focused = false;
        break;
      }
    }
  }

  // TODO: active sources?
}

void OpsiRenderer::SourceChannel::update()
{
  _begin = this->source.delayline.get_read_circulator(this->delay);
  _end = _begin + source.parent.block_size();
}

apf::CombineChannelsResult::type
OpsiRenderer::RenderFunction::select(SourceChannel& in)
{
  _in = &in;

  // define a restricted area around loudspeakers to avoid division by zero:
  const float safety_radius = 0.01f; // 1 cm

  // TODO: move reference calculation to OpsiRenderer::Process?
  auto ref = DirectionalPoint(_out.parent.state.reference_position
      , _out.parent.state.reference_orientation);

  // TODO: this is actually wrong!
  // We use it to be compatible with the (also wrong) GUI implementation.
  auto ref_off = ref;
  ref_off.transform(DirectionalPoint(
        _out.parent.state.reference_offset_position
        , _out.parent.state.reference_offset_orientation));

  sample_type weighting_factor = 1;
  float float_delay = 0;

  auto ls = Loudspeaker(_out);
  auto src_pos = in.source.position;

  // TODO: shortcut if in.source.weighting_factor == 0

  // Transform loudspeaker position according to reference and offset
  ls.transform(ref);

  float reference_distance = (ls.position - ref_off.position).length();

  float source_ls_distance = (ls.position - src_pos).length();

  switch (in.source.model) // check if point source or plane wave or ...
  {
    case ::Source::point:
      if (ls.model == Loudspeaker::subwoofer)
      {
        // the delay is calculated to be correct on the reference position
        // delay can be negative!
        float_delay = (src_pos - ref_off.position).length()
          - reference_distance;

        // setting the subwoofer amplitude to 1 is the unwritten standard 
        // (cf. AAP renderer)
        weighting_factor = 1.0f;

        break; // step out of switch
      }

      float_delay = source_ls_distance;
      assert(float_delay >= 0);

      float denominator;
      if (float_delay < safety_radius) denominator = std::sqrt(safety_radius);
      else denominator = std::sqrt(float_delay);

      // TODO: does this really do the right thing?
      weighting_factor = cos(angle(ls.position - src_pos,
            ls.orientation)) / denominator;

      if (weighting_factor < 0.0f)
      {
        // negative weighting factor is only valid for focused sources
        if (in.source._focused)
        {
          // loudspeaker selection:

          // this could also be done by using the cosine function instead of
          // inner product

          // calculate inner product of those two vectors:
          // this = source
          auto lhs = ls.position - src_pos;
          auto rhs = ref_off.position - src_pos;

          // rhs will be zero if src_pos is exactly at the reference position
          // This would would lead to the inner product being zero.
          // lhs can't be zero (checked before to avoid infinite gain)
          if (rhs.x == 0.0f && rhs.y == 0.0f)
          {
            rhs.y = -0.001;
          }

          // TODO: write inner product function in Position class
          if ((lhs.x * rhs.x + lhs.y * rhs.y) < 0.0f)
          {
            // if the inner product is less than zero, the source is more or
            // less between the loudspeaker and the reference
            float_delay = -float_delay;
            weighting_factor = -weighting_factor;

#if defined(WEIGHTING_OLD)
            (void)source_ls_distance;  // avoid "unused variable" warning
#elif defined(WEIGHTING_DELFT)
            // limit to a maximum of 2.0
            weighting_factor *= std::min(2.0f, std::sqrt(source_ls_distance
                / (reference_distance + source_ls_distance)));
#endif
          }
          else
          {
            // ignored focused point source
            weighting_factor = 0;
            break;
          }
        }
        else // non-focused and weighting_factor < 0
        {
          // ignored non-focused point source
          weighting_factor = 0;
          break;
        }
      }
      else if(weighting_factor > 0.0f) // positive weighting factor
      {
        if (!in.source._focused)
        {
          // non-focused point source

#if defined(WEIGHTING_OLD)
#elif defined(WEIGHTING_DELFT)
          // WARNING: division by zero is possible!
          weighting_factor *= std::sqrt(source_ls_distance
              / (reference_distance + source_ls_distance));
#endif
        }
        else // focused
        {
          // ignored focused point source
          break;
        }
      }
      else
      {
        // this should never happen: Weighting factor is 0 or +-Inf or NaN!
        break;
      }
      break;

    case ::Source::plane:
      if (ls.model == Loudspeaker::subwoofer)
      {
        weighting_factor = 1.0f; // TODO: is this correct?
        // the delay is calculated to be correct on the reference position
        // delay can be negative!
        float_delay
          = DirectionalPoint(in.source.position, in.source.orientation)
          .plane_to_point_distance(ref_off.position) - reference_distance;
        break; // step out of switch
      }

      // weighting factor is determined by the cosine of the angle
      // difference between plane wave direction and loudspeaker direction
      weighting_factor = cos(angle(in.source.orientation, ls.orientation));
      // check if loudspeaker is active for this source
      if (weighting_factor < 0)
      {
        // ignored plane wave
        weighting_factor = 0;
        break;
      }

      float_delay = DirectionalPoint(in.source.position, in.source.orientation)
        .plane_to_point_distance(ls.position);

      if (float_delay < 0.0)
      {
        // "focused" plane wave
      }
      else // positive delay
      {
        // plane wave
      }
      break;

    default:
      //WARNING("Unknown source model");
      break;
  } // switch source model

#if defined(WEIGHTING_OLD)
  if (in.source.model == ::Source::point)
  {
    // compensate for inherent distance decay (approx. 1/sqrt(r))
    // no compensation closer to 0.5 m to the reference
    // this is the same operation for focused and non-focused sources
    // exclude subwoofers as there is no inherent amplitude decay
    if (ls.model != Loudspeaker::subwoofer)
    {
      weighting_factor *= 
        std::sqrt(std::max((src_pos - ref_off.position).length(), 0.5f)); 
    }
  }
#elif defined(WEIGHTING_DELFT)
// TODO: Undo default distance attenuation
#endif
  
  // apply the gain factor of the current source
  weighting_factor *= in.source.weighting_factor;

  // apply tapering
  weighting_factor *= ls.weight;

  assert(weighting_factor >= 0.0f);

  // delay in seconds
  float_delay *= c_inverse;
  // delay in samples
  float_delay *= _out.parent.sample_rate();

  // TODO: check for negative delay and print an error if > initial_delay

  // TODO: do proper rounding
  // TODO: enable interpolated reading from delay line.
  int int_delay = static_cast<int>(float_delay + 0.5f);

  if (in.source.delayline.delay_is_valid(int_delay))
  {
    in.delay = int_delay;
    in.weighting_factor = weighting_factor;
  }
  else
  {
    // TODO: some sort of warning message?

    in.delay = 0;
    in.weighting_factor = 0;
  }

  assert(in.weighting_factor.exactly_one_assignment());
  assert(in.delay.exactly_one_assignment());

  _old_factor = in.weighting_factor.old();
  _new_factor = in.weighting_factor;

  using namespace apf::CombineChannelsResult;
  auto crossfade_mode = apf::CombineChannelsResult::type();

  if (_old_factor == 0 && _new_factor == 0)
  {
    crossfade_mode = nothing;
  }
  else if (_old_factor == _new_factor && !in.delay.changed())
  {
    crossfade_mode = constant;
  }
  else if (_old_factor == 0)
  {
    crossfade_mode = fade_in;
  }
  else if (_new_factor == 0)
  {
    crossfade_mode = fade_out;
  }
  else
  {
    crossfade_mode = change;
  }

  if (crossfade_mode == nothing || crossfade_mode == fade_in)
  {
    // No need to read the delayline here
  }
  else
  {
    in._begin = in.source.delayline.get_read_circulator(in.delay.old());
    in._end = in._begin + _out.parent.block_size();
  }

  return crossfade_mode;
}

}  // namespace ssr

#endif

// Settings for Vim (http://www.vim.org/), please do not remove:
// vim:softtabstop=2:shiftwidth=2:expandtab:textwidth=80:cindent
