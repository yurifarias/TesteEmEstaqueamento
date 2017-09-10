import Jama.Matrix;

public class EstaqueamentoPlano extends AnalisesPreliminares {

    private double fi;
    private double xo;
    private double yo;
    private double zo;

    /* Método para calcular os esforços normais de reação nas estacas: [N] = [v']transposta * [p'] */
    public Matrix reacosNormais(Estaca[] estacas, double[][] esfExternos, char planoSimetria) {

        novasCoordenadas(estacas, planoSimetria);

        return mMovElasticoTransformado(estacas, esfExternos, planoSimetria).transpose().times(mComponentesDeVetorTransformado(estacas, planoSimetria));
    }

    /* Método para achar as coordenadas xo e yo dos novos eixos coordenados x' e y' após a translação
    dos eixos originais x e y, e o ângulo de rotação fi após a rotação em torno do eixo original z,
    como aritifício para solucionar os casos de estaqueamentos planos. */
    private void novasCoordenadas(Estaca[] estacas, char planoSimetria) {

        double[][] mS = mRigidez(estacas).getArray();

        if (planoSimetria == 'B') {

            fi = (Math.atan((2 * mS[0][1]) / (mS[0][0] - mS[1][1]))) / 2;
            xo = (mS[1][5] * mS[0][0] - mS[0][5] * mS[0][1]) / (mS[0][0] * mS[1][1] - Math.pow(mS[0][1], 2));
            yo = (mS[1][5] * mS[0][1] - mS[0][5] * mS[1][1]) / (mS[0][0] * mS[1][1] - Math.pow(mS[0][1], 2));

        } if (planoSimetria == 'C') {

            fi = (Math.atan((2 * mS[0][2]) / (mS[0][0] - mS[2][2]))) / 2;
            xo = (mS[0][4] * mS[0][2] - mS[2][4] * mS[0][0]) / (mS[0][0] * mS[2][2] - Math.pow(mS[0][2], 2));
            zo = (mS[0][4] * mS[2][2] - mS[2][4] * mS[0][2]) / (mS[0][0] * mS[2][2] - Math.pow(mS[0][2], 2));

        }
    }

    /* Método para criar a matriz de trnasformação [T], ou tensor de transformação [T], referente aos
    casos de estaquemaneto plano em XY ou XZ */
    private Matrix tensorTransformacao(char planoSimetria) {

        double[][] tensorT = new double[6][6];

        if (planoSimetria == 'B') {

            tensorT[0][0] = Math.cos(fi);
            tensorT[0][1] = Math.sin(fi);
            tensorT[0][2] = 0;
            tensorT[0][3] = 0;
            tensorT[0][4] = 0;
            tensorT[0][5] = 0;
            tensorT[1][0] = -Math.sin(fi);
            tensorT[1][1] = Math.cos(fi);
            tensorT[1][2] = 0;
            tensorT[1][3] = 0;
            tensorT[1][4] = 0;
            tensorT[1][5] = 0;
            tensorT[2][0] = 0;
            tensorT[2][1] = 0;
            tensorT[2][2] = 1;
            tensorT[2][3] = 0;
            tensorT[2][4] = 0;
            tensorT[2][5] = 0;
            tensorT[3][0] = 0;
            tensorT[3][1] = 0;
            tensorT[3][2] = 0;
            tensorT[3][3] = 1;
            tensorT[3][4] = 0;
            tensorT[3][5] = 0;
            tensorT[4][0] = 0;
            tensorT[4][1] = 0;
            tensorT[4][2] = 0;
            tensorT[4][3] = 0;
            tensorT[4][4] = 1;
            tensorT[4][5] = 0;
            tensorT[5][0] = yo;
            tensorT[5][1] = -xo;
            tensorT[5][2] = 0;
            tensorT[5][3] = 0;
            tensorT[5][4] = 0;
            tensorT[5][5] = 1;

        } if (planoSimetria == 'C') {

            tensorT[0][0] = Math.cos(fi);
            tensorT[0][1] = 0;
            tensorT[0][2] = Math.sin(fi);
            tensorT[0][3] = 0;
            tensorT[0][4] = 0;
            tensorT[0][5] = 0;
            tensorT[1][0] = 0;
            tensorT[1][1] = 1;
            tensorT[1][2] = 0;
            tensorT[1][3] = 0;
            tensorT[1][4] = 0;
            tensorT[1][5] = 0;
            tensorT[2][0] = -Math.sin(fi);
            tensorT[2][1] = 0;
            tensorT[2][2] = Math.cos(fi);
            tensorT[2][3] = 0;
            tensorT[2][4] = 0;
            tensorT[2][5] = 0;
            tensorT[3][0] = 0;
            tensorT[3][1] = 0;
            tensorT[3][2] = 0;
            tensorT[3][3] = 1;
            tensorT[3][4] = 0;
            tensorT[3][5] = 0;
            tensorT[4][0] = -zo;
            tensorT[4][1] = 0;
            tensorT[4][2] = xo;
            tensorT[4][3] = 0;
            tensorT[4][4] = 1;
            tensorT[4][5] = 0;
            tensorT[5][0] = 0;
            tensorT[5][1] = 0;
            tensorT[5][2] = 0;
            tensorT[5][3] = 0;
            tensorT[5][4] = 0;
            tensorT[5][5] = 1;

        } return new Matrix(tensorT);
    }

    /* Método para calcular a matriz de rigidez transformada [S'], aplicando o tensor de transformação [T],
    retornando uma matriz 6 x 6: [S'] = [T] * [S] * [T]transposta */
    private Matrix mRigidezTransformada(Estaca[] estacas, char planoSimetria) {

        return (tensorTransformacao(planoSimetria).times(mRigidez(estacas))).times(tensorTransformacao(planoSimetria)).transpose();
    }

    /* Método para calcular a matriz de esforços transformados [F'], aplicando o tensor de transformação [T],
    retornando uma matraiz 6 x 1: [F'] = [T] * [F] */
    private Matrix esfExternosTransformados(double[][] esfExternos, char planoSimetria) {

        Matrix esfExternosTransformados = tensorTransformacao(planoSimetria).times(new Matrix(esfExternos));
        return esfExternosTransformados;
    }

    /* Método para calcular a matriz do movimento elástico transformado do bloco [v'], após a transformação
    da matriz de rigidez [S'] e da matriz do esforços externos [F'],
    retornando uma matriz 6 x 1: [v'] = [S' ^ -1] * [F'] */
    private Matrix mMovElasticoTransformado(Estaca[] estacas, double[][] esfExternos, char planoSimetria) {

        double[][] vL = new double[6][1];

        if (mRigidezTransformada(estacas, planoSimetria).getArray()[0][0] == 0) {

            vL[0][0] = 0;

        } else {

            vL[0][0] = esfExternosTransformados(esfExternos, planoSimetria).getArray()[0][0] / mRigidezTransformada(estacas, planoSimetria).getArray()[0][0];

        } if (mRigidezTransformada(estacas, planoSimetria).getArray()[1][1] == 0) {

            vL[1][0] = 0;

        } else {

            vL[1][0] = esfExternosTransformados(esfExternos, planoSimetria).getArray()[1][0] / mRigidezTransformada(estacas, planoSimetria).getArray()[1][1];

        } if (mRigidezTransformada(estacas, planoSimetria).getArray()[2][2] == 0) {

            vL[2][0] = 0;

        } else {

            vL[2][0] = esfExternosTransformados(esfExternos, planoSimetria).getArray()[2][0] / mRigidezTransformada(estacas, planoSimetria).getArray()[2][2];

        } if (mRigidezTransformada(estacas, planoSimetria).getArray()[3][3] == 0) {

            vL[3][0] = 0;

        } else {

            vL[3][0] = esfExternosTransformados(esfExternos, planoSimetria).getArray()[3][0] / mRigidezTransformada(estacas, planoSimetria).getArray()[3][3];

        } if (mRigidezTransformada(estacas, planoSimetria).getArray()[4][4] == 0) {

            vL[4][0] = 0;

        } else {

            vL[4][0] = esfExternosTransformados(esfExternos, planoSimetria).getArray()[4][0] / mRigidezTransformada(estacas, planoSimetria).getArray()[4][4];

        } if (mRigidezTransformada(estacas, planoSimetria).getArray()[5][5] == 0) {

            vL[5][0] = 0;

        } else {

            vL[5][0] = esfExternosTransformados(esfExternos, planoSimetria).getArray()[5][0] / mRigidezTransformada(estacas, planoSimetria).getArray()[5][5];

        } return new Matrix(vL);
    }

    /* Método para calcular a matriz de componentes de estaca transformados [p'], aplicando o tensor de
    transformação [T], retornando uma matriz 6 x n: [p'] = [T] * [p],
    onde n é número de estacas */
    private Matrix mComponentesDeVetorTransformado(Estaca[] estacas, char planoSimetria) {

        return tensorTransformacao(planoSimetria).times(mComponentesDeVetor(estacas));
    }

}