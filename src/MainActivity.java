import Jama.Matrix;

import java.util.Arrays;

public class MainActivity {

    public static void main(String[] args) {

        AnalisesPreliminares calcular = new AnalisesPreliminares();

        /* Exemplo de estaqueamento plano em XY */
//        Estaca[] estacas = new Estaca[3];
//
//        estacas[0] = new Estaca(1, 0.3, 0, 10, 0);
//        estacas[1] = new Estaca(1, 0, 0, 0, 0);
//        estacas[2] = new Estaca(1, -0.3, 0, 10, 180);

        /* Exemplo de estaqueamento plano em XZ */
//        Estaca[] estacas = new Estaca[3];
//
//        estacas[0] = new Estaca(1, 0, 0.3, 10, 90);
//        estacas[1] = new Estaca(1, 0, 0, 0, 0);
//        estacas[2] = new Estaca(1, 0, -0.3, 10, 270);

        /*Exemplo de estaqueamento com simetria em relacao ao plano XY */
        Estaca[] estacas = new Estaca[5];

        estacas[0] = new Estaca(1, 0.4, 0.4, 10, 30);
        estacas[1] = new Estaca(1, 0, 0.4, 10, 90);
        estacas[2] = new Estaca(1, -0.4, 0.4, 10, 150);
        estacas[3] = new Estaca(1, -0.3, -0.3, 10, 225);
        estacas[4] = new Estaca(1, 0.3, -0.3, 10, 315);

        /*Exemplo de estaqueamento com simetria em relacao ao plano XY */
//        Estaca[] estacas = new Estaca[5];
//
//        estacas[0] = new Estaca(1, -0.3, -0.3, 10, 210);
//        estacas[1] = new Estaca(1, -0.3, 0.3, 10, 150);
//        estacas[2] = new Estaca(1, 0.3, 0, 10, 0);
//        estacas[3] = new Estaca(1, 0.3, -0.5, 10, 330);
//        estacas[4] = new Estaca(1, 0.3, 0.5, 10, 30);

        /*Exemplo de estaqueamento com simetria em relacao aos planos XY e XZ */
//        Estaca[] estacas = new Estaca[8];
//
//        estacas[0] = new Estaca(1, 0.5, 0.3, 10, 30);
//        estacas[1] = new Estaca(1, -0.5, 0.3, 10, 150);
//        estacas[2] = new Estaca(1, -0.5, -0.3, 10, 210);
//        estacas[3] = new Estaca(1, 0.5, -0.3, 10, 330);
//        estacas[4] = new Estaca(1, 0.2, 0, 10, 0);
//        estacas[5] = new Estaca(1, -0.2, 0, 10, 180);
//        estacas[6] = new Estaca(1, 0, 0.2, 10, 90);
//        estacas[7] = new Estaca(1, 0, -0.2, 10, 270);

        /*Exemplo de estaqueamento com simetria em relacao ao eixo x */
//        Estaca[] estacas = new Estaca[5];
//
//        estacas[0] = new Estaca(1, 0.5, 0.3, 10, 300);
//        estacas[1] = new Estaca(1, 0, 0, 0, 0);
//        estacas[2] = new Estaca(1, -0.5, -0.3, 10, 120);
//        estacas[3] = new Estaca(1, 0.4, 0.2, 12, 50);
//        estacas[4] = new Estaca(1, -0.4, -0.2, 12, 230);

        /* Exemplo de estaqueamento com todas as estacas paralelas */
//        Estaca[] estacas = new Estaca[5];
//
//        estacas[0] = new Estaca(1, 0.4, 0.5, 0, 0);
//        estacas[1] = new Estaca(1, -0.4, 0.5, 10, 180);
//        estacas[2] = new Estaca(1, -0.4, 0.5, 10, 180);
//        estacas[3] = new Estaca(1, 0.4, 0.6, 0, 0);
//        estacas[4] = new Estaca(1,0,0,0,0);


        double[][] esforcos = {{350000}, {0}, {0}, {0}, {0}, {0}};

        char planoSimetria = calcular.tirarSimetria(estacas);

        char casoSimetria = calcular.getPlanoSimetria(estacas);

        System.out.println(casoSimetria);

        if (casoSimetria == 'A') {

            EstacasParalelas estacasParalelas = new EstacasParalelas();

            Matrix normais = estacasParalelas.reacoesNormais(estacas, esforcos);

            System.out.println(Arrays.deepToString(normais.getArray()));

        } else if (casoSimetria == 'B' || casoSimetria == 'C') {

            EstaqueamentoPlano estaqueamentoPlano = new EstaqueamentoPlano();

            Matrix normais = estaqueamentoPlano.reacosNormais(estacas, esforcos, planoSimetria);

            System.out.println(Arrays.deepToString(normais.getArray()));

        } else if (casoSimetria == 'D' || casoSimetria == 'E') {

            SimetriaEmUmPlano simetriaEmUmPlano = new SimetriaEmUmPlano();

            Matrix normais = simetriaEmUmPlano.reacosNormais(estacas, esforcos, planoSimetria);

            System.out.println(Arrays.deepToString(normais.getArray()));

        } else if (casoSimetria == 'F') {

            SimetriaEmDoisPlanos simetriaEmDoisPlanos = new SimetriaEmDoisPlanos();

            Matrix normais = simetriaEmDoisPlanos.reacoesNormais(estacas, esforcos);

            System.out.println(Arrays.deepToString(normais.getArray()));

        }  else if (casoSimetria == 'G') {

            SimetriaPorUmEixo simetriaPorUmEixo = new SimetriaPorUmEixo();

            Matrix normais = simetriaPorUmEixo.reacoesNormais(estacas, esforcos);

            System.out.println(Arrays.deepToString(normais.getArray()));

        } else if (casoSimetria == 'Z') {

            EstaqueamentoGeral estaqueamentoGeral = new EstaqueamentoGeral();

            Matrix normais = estaqueamentoGeral.reacoesNormais(estacas, esforcos);

            System.out.println(Arrays.deepToString(normais.getArray()));

        }
    }
}