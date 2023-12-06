// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// SECDED decoder generated by util/design/secded_gen.py

module prim_secded_16_11_dec (
  input        [15:0] data_i,
  output logic [10:0] data_o,
  output logic [10:0] data_orig_o,
  output logic [4:0] syndrome_o,
  output logic [1:0] err_o
);

  always_comb begin : p_encode
    // Syndrome calculation
    syndrome_o[0] = ^(data_i & 16'h0C3F);
    syndrome_o[1] = ^(data_i & 16'h15C7);
    syndrome_o[2] = ^(data_i & 16'h26D9);
    syndrome_o[3] = ^(data_i & 16'h476A);
    syndrome_o[4] = ^(data_i & 16'h87B4);

    // Corrected output calculation
    data_o[0] = (syndrome_o == 5'h7) ^ data_i[0];
    data_o[1] = (syndrome_o == 5'hb) ^ data_i[1];
    data_o[2] = (syndrome_o == 5'h13) ^ data_i[2];
    data_o[3] = (syndrome_o == 5'hd) ^ data_i[3];
    data_o[4] = (syndrome_o == 5'h15) ^ data_i[4];
    data_o[5] = (syndrome_o == 5'h19) ^ data_i[5];
    data_o[6] = (syndrome_o == 5'he) ^ data_i[6];
    data_o[7] = (syndrome_o == 5'h16) ^ data_i[7];
    data_o[8] = (syndrome_o == 5'h1a) ^ data_i[8];
    data_o[9] = (syndrome_o == 5'h1c) ^ data_i[9];
    data_o[10] = (syndrome_o == 5'h1f) ^ data_i[10];

    data_orig_o = data_i;

    // err_o calc. bit0: single error, bit1: double error
    err_o[0] = ^syndrome_o;
    err_o[1] = ~err_o[0] & (|syndrome_o);
  end
endmodule : prim_secded_16_11_dec
