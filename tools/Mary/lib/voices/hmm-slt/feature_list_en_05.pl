#!/usr/bin/perl

## This structure is common to make_label_mono.pl, make_label_full.pl
#  and make_questions.pl, and is imported via scripts/common_routines.pl

# Requested features for the fullcontext names and tree questions
# (apart from phonological features such as phoneme ID, vc, cplace etc.)
#
# Comment out the ones you don't want.
#
# changed: mary_pos (DE) by mary_gpos (EN)
#
%requested_fea_keys = ( #"mary_stressed", "b",
                        "mary_pos_in_syl", "s",
                        "mary_syl_break", "v",
                        "mary_prev_syl_break", "v",
                        "mary_position_type", "v",
                        #"mary_next_is_pause", "b",
                        #"mary_prev_is_pause", "b",
                        "mary_gpos", "v",
                        #"mary_tobi_accent", "v",
                        #"mary_tobi_endtone", "v",
                        #"mary_next_tobi_accent", "v",
                        #"mary_next_tobi_endtone", "v",
                        #"mary_nextnext_tobi_accent", "v",
                        #"mary_nextnext_tobi_endtone", "v",
                        #"mary_sentence_punc", "v",
                        #"mary_accented", "b",
                        #"mary_sentence_numphrases", "s",
                        #"mary_phrases_from_sentence_start", "s",
                        #"mary_phrases_from_sentence_end", "s",
                        #"mary_sentence_numwords", "s",
                        #"mary_words_from_sentence_start", "s",
                        #"mary_words_from_sentence_end", "s",
                        #"mary_phrase_numwords", "s",
                        #"mary_words_from_phrase_start", "s",
                        #"mary_words_from_phrase_end", "s",
                        #"mary_phrase_numsyls", "s",
                        #"mary_syls_from_phrase_start", "s",
                        #"mary_syls_from_phrase_end", "s",
                        #"mary_stressed_syls_from_phrase_start", "s",
                        #"mary_stressed_syls_from_phrase_end", "s",
                        #"mary_accented_syls_from_phrase_start", "s",
                        #"mary_accented_syls_from_phrase_end", "s",
                        #"mary_word_numsyls", "s",
                        #"mary_syls_from_word_start", "s",
                        #"mary_syls_from_word_end", "s",
                        #"mary_word_numsegs", "s",
                        #"mary_segs_from_word_start", "s",
                        #"mary_segs_from_word_end", "s",
                        #"mary_syl_numsegs", "s",
                        #"mary_segs_from_syl_start", "s",
                        #"mary_segs_from_syl_end", "s",
                        #"mary_syls_from_prev_stressed", "s",
                        #"mary_syls_to_next_stressed", "s",
                        #"mary_syls_from_prev_accent", "s",
                        #"mary_syls_to_next_accent", "s",
                        #"mary_phrase_endtone", "v",
                        #"mary_prev_phrase_endtone", "v",
                        #"mary_next_accent", "v",
                        #"mary_prev_accent", "v",
                        #"mary_prev_punctuation", "v",
                        #"mary_next_punctuation", "v",
                        #"mary_words_from_prev_punctuation", "s",
                        #"mary_words_to_next_punctuation", "s",
                        #"mary_word_frequency", "s" 
		);

1;

